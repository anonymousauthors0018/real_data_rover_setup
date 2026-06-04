import os
import json
import time
import threading
import traceback
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from radar_msgs.msg import RadarFrameRef
from std_srvs.srv import Trigger

from mmwave.dataloader import DCA1000
from mmwave.dataloader.radars import TI


def now_ns(node: Node) -> int:
    # ROS time in nanoseconds (Foxy)
    return int(node.get_clock().now().nanoseconds)


def ensure_dir(p: str) -> str:
    Path(p).mkdir(parents=True, exist_ok=True)
    return p


class RadarCaptureNode(Node):
    """
    Radar capture node:
      - configure radar + DCA
      - start framing
      - capture ADC batches
      - write raw .bin and processed .npy
      - publish JSON refs with ROS timestamps
    """

    def __init__(self):
        super().__init__("radar_capture_node")

        # ----- Parameters -----
        self.declare_parameter("dca_config_file", "/workspace/pyRadar/configFiles/cf.json")
        self.declare_parameter("radar_config_file", "/workspace/pyRadar/configFiles/xWR1843Boost_config_matlab.cfg")

        self.declare_parameter("cli_port", "/dev/ttyACM0")
        self.declare_parameter("data_port", "/dev/ttyACM1")
        self.declare_parameter("cli_baud", 115200)
        self.declare_parameter("data_baud", 921600)

        self.declare_parameter("frame_num_in_buf", 32)        # DCA thread buffer depth
        self.declare_parameter("numframes_per_read", 1)       # publish/write frequency control
        self.declare_parameter("sort_in_c", True)

        # Output folders (mount a volume in Docker, e.g. -v /data:/data)
        self.declare_parameter("out_root", "/workspace/radar_data")
        self.declare_parameter("raw_dir", "raw")
        self.declare_parameter("proc_dir", "proc")

        # Processing mode:
        #   "none" -> only write raw bin
        #   "range_doppler_mag" -> save a lightweight processed array
        #   "custom" -> call your own postProc and save what you want
        self.declare_parameter("proc_mode", "none")

        # ----- Publishers -----
        self.pub_status = self.create_publisher(String, "/radar/status", 10)
        self.pub_frame_ref = self.create_publisher(RadarFrameRef, "/radar/frame_ref", 50)

        # ----- Services -----
        self.srv_configure = self.create_service(Trigger, "/radar/configure", self.on_configure)
        self.srv_start = self.create_service(Trigger, "/radar/start", self.on_start)
        self.srv_stop = self.create_service(Trigger, "/radar/stop", self.on_stop)

        # ----- State -----
        self._dca: Optional[DCA1000] = None
        self._radar: Optional[TI] = None
        self._adc_params: Optional[dict] = None

        self._configured = False
        self._running = False

        self._stop_evt = threading.Event()
        self._worker: Optional[threading.Thread] = None

        self._seq = 0

        self._publish_status("idle (call /radar/configure)")

    # -------------------------
    # ROS callbacks
    # -------------------------
    def on_configure(self, req: Trigger.Request, resp: Trigger.Response):
        try:
            if self._configured:
                resp.success = True
                resp.message = "Already configured"
                return resp

            self._do_configure()
            self._configured = True
            resp.success = True
            resp.message = "Configured OK"
            self._publish_status("configured")
            return resp

        except Exception:
            self.get_logger().error("Configure failed:\n" + traceback.format_exc())
            resp.success = False
            resp.message = "Configure failed (see logs)"
            self._publish_status("configure_failed")
            return resp

    def on_start(self, req: Trigger.Request, resp: Trigger.Response):
        try:
            if not self._configured:
                resp.success = False
                resp.message = "Not configured. Call /radar/configure first."
                return resp

            if self._running:
                resp.success = True
                resp.message = "Already running"
                return resp

            self._do_start()
            resp.success = True
            resp.message = "Started"
            self._publish_status("running")
            return resp

        except Exception:
            self.get_logger().error("Start failed:\n" + traceback.format_exc())
            resp.success = False
            resp.message = "Start failed (see logs)"
            self._publish_status("start_failed")
            return resp

    def on_stop(self, req: Trigger.Request, resp: Trigger.Response):
        try:
            if not self._running:
                resp.success = True
                resp.message = "Already stopped"
                self._publish_status("configured")
                return resp

            self._do_stop()
            resp.success = True
            resp.message = "Stopped"
            self._publish_status("configured")
            return resp

        except Exception:
            self.get_logger().error("Stop failed:\n" + traceback.format_exc())
            resp.success = False
            resp.message = "Stop failed (see logs)"
            self._publish_status("stop_failed")
            return resp

    # -------------------------
    # Internal operations
    # -------------------------
    def _publish_status(self, s: str):
        msg = String()
        msg.data = s
        self.pub_status.publish(msg)

    def _do_configure(self):
        dca_config_file = self.get_parameter("dca_config_file").value
        radar_config_file = self.get_parameter("radar_config_file").value

        cli_port = self.get_parameter("cli_port").value
        data_port = self.get_parameter("data_port").value
        cli_baud = int(self.get_parameter("cli_baud").value)
        data_baud = int(self.get_parameter("data_baud").value)

        # Create DCA
        self._dca = DCA1000()

        # Reset both
        self._dca.reset_radar()
        self._dca.reset_fpga()
        time.sleep(1.0)

        self._dca.refresh_parameter()
        # Create Radar UART controller
        self._radar = TI(
            cli_loc=cli_port,
            cli_baud=cli_baud,
            data_loc=data_port,
            data_baud=data_baud,
            config_file=radar_config_file,
            verbose=True,
        )

        # Infinite frames configured by cfg (or use setFrameCfg in cfg)
        self._radar.setFrameCfg(0)

        # Configure FPGA/record using your helper
        adc_params, _ = self._dca.configure(dca_config_file, radar_config_file)
        self._adc_params = adc_params
        time.sleep(2)
        self.get_logger().info("Configured radar + DCA OK")

    def _do_start(self):
        assert self._dca is not None and self._radar is not None
    
        frame_num_in_buf = int(self.get_parameter("frame_num_in_buf").value)

        self._stop_evt.clear()

        # Start DCA streaming and UDP receiver thread
        self._dca.stream_start()
        self._dca.fastRead_in_Cpp_thread_start(frame_num_in_buf)

        # Start radar sensor framing
        self._radar.startSensor()

        # Start worker thread
        self._running = True
        self._worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker.start()

        self.get_logger().info("Radar framing started")

    def _do_stop(self):
        self._stop_evt.set()

        # Stop worker
        if self._worker is not None:
            self._worker.join(timeout=2.0)
            self._worker = None

        # Stop radar + DCA
        if self._radar is not None:
            try:
                self._radar.stopSensor()
            except Exception:
                pass

        if self._dca is not None:
            try:
                self._dca.fastRead_in_Cpp_thread_stop()
            except Exception:
                pass
            try:
                self._dca.stream_stop()
            except Exception:
                pass

        self._running = False
        self.get_logger().info("Radar stopped")

    def _capture_loop(self):
        assert self._dca is not None

        out_root = self.get_parameter("out_root").value


        numframes = int(self.get_parameter("numframes_per_read").value)
        sort_in_c = bool(self.get_parameter("sort_in_c").value)
        proc_mode = self.get_parameter("proc_mode").value

        session = time.strftime("%Y%m%d_%H%M%S")
        # session_dir = os.path.join(out_root, session)
        raw_dir = ensure_dir(os.path.join(out_root, self.get_parameter("raw_dir").value))
        # proc_dir = ensure_dir(os.path.join(out_root, self.get_parameter("proc_dir").value))

        self.get_logger().info(f"Capture session: {session}")
        self.get_logger().info(f"Saving data to: {out_root}")
        first_frame_sent = False

        try:
            while not self._stop_evt.is_set():
                # Blocking read until numframes are available
                # data_buf = self._dca.fastRead_in_Cpp_thread_get(
                #     numframes, verbose=False, sortInC=sort_in_c
                # )


                seq = self._seq
                self._seq += 1

                raw_path = os.path.join(raw_dir, f"raw_{session}_seq{seq:06d}.bin")
                # proc_path = os.path.join(proc_dir, f"proc_{session}_seq{seq:06d}.npy")

                # t_ns = now_ns(self)  # ROS time when batch assembled
                t_ns = self.get_clock().now()
                data_buf = self._dca.fastRead_in_Cpp_thread_get(numframes, verbose=False, sortInC=False)
                # Write raw ADC
                data_buf.tofile(raw_path)

                # if seq % 10 == 0:
                #     self.get_logger().info(
                #         f"timing: read={t1-t0:.3f}s write={t3-t2:.3f}s total={t3-t0:.3f}s"
                #     )

                raw_bytes = os.path.getsize(raw_path)

                proc_info = None
                if proc_mode == "none":
                    proc_path = ""
                elif proc_mode == "range_doppler_mag":
                    # Lightweight processed output example:
                    # Save a magnitude summary so files stay manageable.
                    # Adjust to your desired processed representation.
                    proc_arr = self._range_doppler_mag(data_buf)
                    np.save(proc_path, proc_arr)
                    proc_info = {"shape": list(proc_arr.shape), "dtype": str(proc_arr.dtype)}
                elif proc_mode == "custom":
                    # Call your own processing function here and save it.
                    # Replace _custom_postproc with your postProc.
                    proc_arr = self._custom_postproc(data_buf)
                    np.save(proc_path, proc_arr)
                    proc_info = {"shape": list(proc_arr.shape), "dtype": str(proc_arr.dtype)}
                elif proc_mode == "dsp_pointcloud": 
                    dsp_processed_data = self._radar.post_process_data_buf(verbose=False)
                    np.save(proc_path, dsp_processed_data)
                    proc_info = {"type": "dsp_pointcloud", "note": "load with allow_pickle=True"}
                else:
                    self.get_logger().warn(f"Unknown proc_mode={proc_mode}, writing raw only")
                    proc_path = ""


                msg = RadarFrameRef()
                msg.header.stamp = t_ns.to_msg()
                msg.header.frame_id = "radar"
                msg.seq = seq
                msg.numframes = numframes
                msg.raw_bin = raw_path
                msg.raw_bytes = raw_bytes
                msg.proc_npy = proc_path if proc_path else ""
                msg.proc_info_json = json.dumps(proc_info) if proc_info else ""
                
                self.pub_frame_ref.publish(msg)

                if not first_frame_sent:
                    self._publish_status("running_first_frame_emitted")
                    first_frame_sent = True

        except Exception:
            self.get_logger().error("Capture loop crashed:\n" + traceback.format_exc())
            self._publish_status("running_crashed")

    # -------------------------
    # Processing helpers (you can replace)
    # -------------------------
    def _range_doppler_mag(self, adc: np.ndarray) -> np.ndarray:
        """
        Minimal example processed output:
        Produces a small summary array that is stable and quick.
        This is NOT a full TI pipeline, just an example that works everywhere.

        adc: numpy array returned by your mmwave capture lib (likely int16)
        """
        # If your adc is a flat int16 vector, you can do a simple FFT magnitude
        x = adc.astype(np.float32)
        if x.ndim > 1:
            x = x.reshape(-1)

        # Chunk for a stable-length FFT (example)
        n = min(len(x), 4096)
        if n < 256:
            return np.zeros((1,), dtype=np.float32)

        x = x[:n]
        spec = np.fft.rfft(x)
        mag = np.abs(spec).astype(np.float32)
        return mag

    def _custom_postproc(self, data_buf: np.ndarray) -> np.ndarray:
        """
        Put your own postProc logic here.
        Return a numpy array you want to store as .npy.
        """
        # Placeholder: save raw as int16 array (still large)
        return np.array(data_buf, copy=True)

    def destroy_node(self):
        # Ensure clean stop on shutdown
        try:
            if self._running:
                self._do_stop()
        except Exception:
            pass

        # Close handles
        try:
            if self._dca is not None:
                self._dca.close()
        except Exception:
            pass

        try:
            if self._radar is not None:
                # Close ports if your library exposes them
                try:
                    self._radar.cli_port.close()
                except Exception:
                    pass
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = RadarCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()