// Bench-only transport test: synthetic 320-range packets on a separate topic.
// No motor pins, cmd_vel subscription or real LiDAR data. Restore xe2 afterwards.
#include <micro_ros_arduino.h>
#include <uxr/client/config.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "wifi_config.h"  // local, gitignored; copied from firmware/xe2
#if UXR_CONFIG_CUSTOM_TRANSPORT_MTU < 1400
#error "320-range transport probe requires the rebuilt micro-ROS library"
#endif
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t pub;
static sensor_msgs__msg__LaserScan scan;
static float ranges[320];
static char frame[] = "synthetic_mtu_probe";
static unsigned sent = 0, errors = 0;
void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, 8888);
  while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) delay(200);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK ||
      rclc_node_init_default(&node, "mtu_probe", "/bench_mtu", &support) != RCL_RET_OK ||
      rclc_publisher_init_best_effort(&pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan_synthetic") != RCL_RET_OK) {
    Serial.println("PROBE INIT FAILED");
    while (true) delay(1000);
  }
  sensor_msgs__msg__LaserScan__init(&scan);
  scan.header.frame_id.data = frame;
  scan.header.frame_id.size = sizeof(frame)-1;
  scan.header.frame_id.capacity = sizeof(frame);
  scan.ranges.data = ranges; scan.ranges.size = scan.ranges.capacity = 320;
  scan.angle_min = 0; scan.angle_increment = 6.283185307f/320;
  scan.angle_max = 6.283185307f-scan.angle_increment;
  scan.range_min = .1f; scan.range_max = 8.f; scan.scan_time = .125f;
  for (int i=0;i<320;i++) ranges[i] = .5f+.001f*i;
  Serial.printf("PROBE START: synthetic only, MTU=%d, bins=320\n", UXR_CONFIG_CUSTOM_TRANSPORT_MTU);
}
void loop() {
  if (rcl_publish(&pub, &scan, NULL) == RCL_RET_OK) sent++; else errors++;
  if ((sent+errors)%8 == 0) Serial.printf("PROBE sent=%u errors=%u\n",sent,errors);
  delay(125);
}
