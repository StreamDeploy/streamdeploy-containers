#!/usr/bin/env bash
set -euo pipefail

# Default environment variables
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedx_cpp}"
: "${METRICS_PORT:=8080}"
: "${PROMETHEUS_PORT:=9090}"
: "${LOG_LEVEL:=INFO}"
: "${COLLECT_SYSTEM_METRICS:=true}"
: "${COLLECT_ROS_METRICS:=true}"

# Create necessary directories
mkdir -p /opt/openmind/telemetry/config /opt/openmind/telemetry/logs

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

echo "Starting OpenMind Telemetry with:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  METRICS_PORT: ${METRICS_PORT}"
echo "  PROMETHEUS_PORT: ${PROMETHEUS_PORT}"
echo "  LOG_LEVEL: ${LOG_LEVEL}"
echo "  COLLECT_SYSTEM_METRICS: ${COLLECT_SYSTEM_METRICS}"
echo "  COLLECT_ROS_METRICS: ${COLLECT_ROS_METRICS}"

# Generate telemetry server Python script
TELEMETRY_SERVER="/opt/openmind/telemetry_server.py"
cat > "${TELEMETRY_SERVER}" <<'EOF'
#!/usr/bin/env python3
import os
import json
import time
import threading
import psutil
import logging
from flask import Flask, jsonify
from prometheus_client import Counter, Gauge, generate_latest, CONTENT_TYPE_LATEST
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

# Configure logging
log_level = os.getenv('LOG_LEVEL', 'INFO')
logging.basicConfig(level=getattr(logging, log_level.upper()))
logger = logging.getLogger(__name__)

# Prometheus metrics
cpu_usage = Gauge('robot_cpu_usage_percent', 'CPU usage percentage')
memory_usage = Gauge('robot_memory_usage_percent', 'Memory usage percentage')
disk_usage = Gauge('robot_disk_usage_percent', 'Disk usage percentage')
gpu_usage = Gauge('robot_gpu_usage_percent', 'GPU usage percentage')
gpu_memory = Gauge('robot_gpu_memory_usage_percent', 'GPU memory usage percentage')
ros_node_count = Gauge('ros_node_count', 'Number of active ROS nodes')
uptime_seconds = Gauge('robot_uptime_seconds', 'System uptime in seconds')

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('openmind_telemetry')
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.status_pub = self.create_publisher(
            String, '/telemetry/status', 10)
        
        # Timer for periodic publishing
        self.timer = self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info('OpenMind Telemetry Node initialized')
    
    def publish_diagnostics(self):
        # Publish system status
        status_msg = String()
        status_data = {
            'timestamp': time.time(),
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

class MetricsCollector:
    def __init__(self):
        self.running = True
        self.collect_system = os.getenv('COLLECT_SYSTEM_METRICS', 'true').lower() == 'true'
        self.collect_ros = os.getenv('COLLECT_ROS_METRICS', 'true').lower() == 'true'
    
    def collect_system_metrics(self):
        while self.running:
            try:
                # CPU metrics
                cpu_usage.set(psutil.cpu_percent(interval=1))
                
                # Memory metrics
                memory_info = psutil.virtual_memory()
                memory_usage.set(memory_info.percent)
                
                # Disk metrics
                disk_info = psutil.disk_usage('/')
                disk_usage.set(disk_info.percent)
                
                # GPU metrics (if available)
                try:
                    import pynvml
                    pynvml.nvmlInit()
                    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                    gpu_util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                    gpu_mem = pynvml.nvmlDeviceGetMemoryInfo(handle)
                    
                    gpu_usage.set(gpu_util.gpu)
                    gpu_memory.set(gpu_mem.used / gpu_mem.total * 100)
                except:
                    # GPU not available or NVIDIA drivers not installed
                    pass
                
                # Uptime
                uptime_seconds.set(time.time() - psutil.boot_time())
                
                time.sleep(5)
                
            except Exception as e:
                logger.error(f"Error collecting system metrics: {e}")
                time.sleep(5)
    
    def collect_ros_metrics(self):
        while self.running:
            try:
                # This would normally connect to ROS and collect node info
                # For now, set a placeholder
                ros_node_count.set(0)
                time.sleep(10)
                
            except Exception as e:
                logger.error(f"Error collecting ROS metrics: {e}")
                time.sleep(10)

# Flask app for health and metrics endpoints
app = Flask(__name__)

@app.route('/health')
def health():
    return jsonify({
        'status': 'healthy',
        'timestamp': time.time(),
        'service': 'openmind-telemetry'
    })

@app.route('/metrics')
def metrics():
    return generate_latest(), 200, {'Content-Type': CONTENT_TYPE_LATEST}

@app.route('/status')
def status():
    return jsonify({
        'cpu_percent': psutil.cpu_percent(),
        'memory_percent': psutil.virtual_memory().percent,
        'disk_percent': psutil.disk_usage('/').percent,
        'uptime_seconds': time.time() - psutil.boot_time(),
        'timestamp': time.time()
    })

def run_flask_app():
    port = int(os.getenv('METRICS_PORT', 8080))
    app.run(host='0.0.0.0', port=port, debug=False)

def run_ros_node():
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    # Start metrics collector
    collector = MetricsCollector()
    
    # Start system metrics collection thread
    if collector.collect_system:
        system_thread = threading.Thread(target=collector.collect_system_metrics, daemon=True)
        system_thread.start()
    
    # Start ROS metrics collection thread
    if collector.collect_ros:
        ros_metrics_thread = threading.Thread(target=collector.collect_ros_metrics, daemon=True)
        ros_metrics_thread.start()
    
    # Start Flask app in background thread
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()
    
    logger.info("OpenMind Telemetry Server started")
    
    # Run ROS node in main thread
    try:
        run_ros_node()
    except KeyboardInterrupt:
        logger.info("Shutting down telemetry server")
        collector.running = False

if __name__ == '__main__':
    main()
EOF

chmod +x "${TELEMETRY_SERVER}"

# Set environment variables
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export METRICS_PORT
export PROMETHEUS_PORT
export LOG_LEVEL
export COLLECT_SYSTEM_METRICS
export COLLECT_ROS_METRICS

# Launch the telemetry server
echo "Launching OpenMind Telemetry Server..."
exec python3 "${TELEMETRY_SERVER}"
