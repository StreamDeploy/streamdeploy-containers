#!/usr/bin/env bash
set -euo pipefail

: "${SINK_URL:=}"
: "${HOSTNAME:=edge-node}"
: "${FB_OUT_FORMAT:=json}"
: "${EXTRA_LABELS:=}"

cat > /fluent-bit/etc/fluent-bit.conf <<'CONF'
[SERVICE]
    Daemon    Off
    HTTP_Server On
    HTTP_Listen 0.0.0.0
    HTTP_Port  2020

[INPUT]
    Name tail
    Path /var/log/*.log,/var/log/syslog
    Tag  sys.*
    Parser docker

[INPUT]
    Name cpu
    Tag  metrics.cpu

[INPUT]
    Name mem
    Tag  metrics.mem

[INPUT]
    Name disk
    Tag  metrics.disk

[INPUT]
    Name netif
    Tag  metrics.net

[INPUT]
    Name   systemd
    Tag    ros.*
    Systemd_Filter _COMM=ros2
    DB     /var/log/ros_systemd.sqlite
CONF

if [[ -n "${SINK_URL}" ]]; then
cat >> /fluent-bit/etc/fluent-bit.conf <<CONF
[OUTPUT]
    Name  http
    Match *
    Host  $(echo "${SINK_URL}" | sed -E 's#https?://([^/:]+).*#\\1#')
    URI   /$(echo "${SINK_URL}" | sed -E 's#https?://[^/]+/?##')
    Format ${FB_OUT_FORMAT}
    Header X-Edge-Host ${HOSTNAME}
    Header X-Labels ${EXTRA_LABELS}
CONF
fi

exec /fluent-bit/bin/fluent-bit -c /fluent-bit/etc/fluent-bit.conf
