#!/usr/bin/env bash
set -euo pipefail
: "${SINK_URL:=}"
mkdir -p /etc/vector
cat > /etc/vector/vector.yaml <<'YML'
sources:
  docker_logs:
    type: docker_logs
    include_containers: ["*"]
transforms:
  to_json:
    type: remap
    inputs: [docker_logs]
    source: |
      .hostname = get_env!("HOSTNAME")
      if exists(.message) && is_string(.message) {
        .parsed, err = parse_json(.message)
        if err == null { . = .parsed }
      }
sinks:
  http_out:
    type: http
    inputs: [to_json]
    uri: "${SINK_URL}"
    encoding:
      codec: json
YML
exec /usr/local/bin/vector -c /etc/vector/vector.yaml -w /var/lib/vector
