#!/usr/bin/env python3
import os
import signal
import subprocess
import time

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

import rclpy
from rclpy.node import Node
from next_ros2ws_interfaces.srv import SetStackMode


class StackManager(Node):
    def __init__(self):
        super().__init__("stack_manager")

        self.srv = self.create_service(SetStackMode, "/stack/set_mode", self.set_mode_callback)

        self.current_mode = "stopped"
        self.nav_proc = None
        self.slam_proc = None

        # Repo where install/setup.bash + nav2_launch_wrapper.sh live
        self.repo_dir = "/home/aun/Downloads/smr300l_gazebo_ros2control-main"

    def set_mode_callback(self, request, response):
        mode = request.mode.lower().strip()
        self.get_logger().info(f"Requested mode: {mode}")

        try:
            if mode == "nav":
                self._stop_slam()
                self._start_nav()
                self.current_mode = "nav"
                response.ok = True
                response.message = "Switched to NAV mode."

            elif mode == "slam":
                self._stop_nav()
                self._start_slam()
                self.current_mode = "slam"
                response.ok = True
                response.message = "Switched to SLAM mode."

            elif mode == "stop":
                self._stop_nav()
                self._stop_slam()
                self.current_mode = "stopped"
                response.ok = True
                response.message = "Stopped all stacks."

            else:
                response.ok = False
                response.message = f"Unknown mode: {mode}"

        except Exception as e:
            response.ok = False
            response.message = f"Error: {str(e)}"

        return response

    def _start_nav(self):
        if self.nav_proc is None or self.nav_proc.poll() is not None:
            self.get_logger().info("Starting Nav2 stack via nav2_launch_wrapper.sh...")

            cmd = "source install/setup.bash && ./nav2_launch_wrapper.sh use_sim_time:=true"
            self.nav_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                cwd=self.repo_dir,
                start_new_session=True,
            )

    def _kill_process_tree(self, pid, timeout=5):
        """Kill a process and all its descendants recursively"""
        if PSUTIL_AVAILABLE:
            try:
                parent = psutil.Process(pid)
                children = parent.children(recursive=True)
                
                # First, try graceful termination
                for child in children:
                    try:
                        child.terminate()
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                
                try:
                    parent.terminate()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
                
                # Wait for processes to terminate
                gone, alive = psutil.wait_procs(children + [parent], timeout=timeout)
                
                # Force kill any remaining processes
                for proc in alive:
                    try:
                        self.get_logger().warn(f"Force killing process {proc.pid} ({proc.name()})")
                        proc.kill()
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                
                # Wait one more time to ensure cleanup
                psutil.wait_procs(alive, timeout=2)
                return
                
            except psutil.NoSuchProcess:
                # Process already dead
                return
            except Exception as e:
                self.get_logger().error(f"Error killing process tree with psutil {pid}: {e}")
        
        # Fallback: use process group kill and pkill for child processes
        try:
            # Try to find and kill child processes using pgrep
            try:
                # Find all child PIDs
                result = subprocess.run(
                    ['pgrep', '-P', str(pid)],
                    capture_output=True,
                    timeout=1
                )
                if result.returncode == 0:
                    child_pids = [int(p) for p in result.stdout.decode().strip().split('\n') if p]
                    for child_pid in child_pids:
                        try:
                            os.kill(child_pid, signal.SIGTERM)
                        except (ProcessLookupError, OSError):
                            pass
            except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
                pass
            
            # Kill the process group (includes all children in the same group)
            try:
                pgid = os.getpgid(pid)
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1)
                # Force kill if still alive
                try:
                    os.kill(pid, 0)  # Check if process exists
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    pass
            except (ProcessLookupError, OSError) as e:
                # Process might already be dead, try direct kill
                try:
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(0.5)
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    pass
        except Exception as e:
            self.get_logger().error(f"Error in fallback process kill {pid}: {e}")

    def _stop_nav(self):
        if self.nav_proc is None:
            return
        
        if self.nav_proc.poll() is not None:
            # Process already dead
            self.nav_proc = None
            return
        
        self.get_logger().info("Stopping Nav2 stack...")
        try:
            # Kill the entire process tree
            self._kill_process_tree(self.nav_proc.pid, timeout=8)
            
            # Wait for the main process
            try:
                self.nav_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Nav2 process didn't exit, forcing cleanup...")
                try:
                    self.nav_proc.kill()
                    self.nav_proc.wait(timeout=1)
                except:
                    pass
            
        except Exception as e:
            self.get_logger().error(f"Error stopping Nav2: {e}")
            # Last resort: try to kill the process group
            try:
                os.killpg(os.getpgid(self.nav_proc.pid), signal.SIGKILL)
            except:
                pass
        finally:
            self.nav_proc = None
            self.get_logger().info("Nav2 stack stopped")

    def _start_slam(self):
        if self.slam_proc is None or self.slam_proc.poll() is not None:
            self.get_logger().info("Starting SLAM Toolbox...")

            cmd = "source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true"
            self.slam_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                cwd=self.repo_dir,
                start_new_session=True,
            )

    def _stop_slam(self):
        if self.slam_proc is None:
            return
        
        if self.slam_proc.poll() is not None:
            # Process already dead
            self.slam_proc = None
            return
        
        self.get_logger().info("Stopping SLAM Toolbox...")
        try:
            # Kill the entire process tree
            self._kill_process_tree(self.slam_proc.pid, timeout=8)
            
            # Wait for the main process
            try:
                self.slam_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("SLAM process didn't exit, forcing cleanup...")
                try:
                    self.slam_proc.kill()
                    self.slam_proc.wait(timeout=1)
                except:
                    pass
            
        except Exception as e:
            self.get_logger().error(f"Error stopping SLAM: {e}")
            # Last resort: try to kill the process group
            try:
                os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGKILL)
            except:
                pass
        finally:
            self.slam_proc = None
            self.get_logger().info("SLAM Toolbox stopped")


def main(args=None):
    rclpy.init(args=args)
    node = StackManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Already shut down


if __name__ == "__main__":
    main()
