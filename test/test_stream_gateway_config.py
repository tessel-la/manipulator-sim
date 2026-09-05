from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def test_stream_gateway_uses_native_wrist_camera_and_mobile_ice_fallbacks():
    media = yaml.safe_load((REPOSITORY_ROOT / "config" / "mediamtx.yml").read_text())
    stream = media["paths"]["manipulator_wrist_camera"]

    assert media["api"] is True
    assert media["apiAddress"] == "127.0.0.1:9997"
    assert media["webrtcLocalUDPAddress"] == ":8189"
    assert media["webrtcLocalTCPAddress"] == ":8189"
    assert "$MJPEG_SOURCE_URL" in stream["runOnInit"]
    assert "libx264" in stream["runOnInit"]
    assert stream["runOnInitRestart"] is True


def test_compose_webrtc_profile_wires_turn_discovery_and_ros_source():
    compose = yaml.safe_load((REPOSITORY_ROOT / "docker-compose.yml").read_text())
    services = compose["services"]
    gateway = services["manipulator_stream_gateway"]
    turn = services["manipulator_turn"]

    assert gateway["profiles"] == ["webrtc"]
    assert turn["profiles"] == ["webrtc"]
    assert gateway["environment"]["MTX_RTPADDRESS"] == ":${STREAM_RTP_PORT:-8000}"
    assert gateway["environment"]["MTX_RTCPADDRESS"] == ":${STREAM_RTCP_PORT:-8001}"
    assert "?transport=tcp" in gateway["environment"]["MTX_WEBRTCICESERVERS2_0_URL"]
    assert gateway["environment"]["MTX_WEBRTCICESERVERS2_0_USERNAME"] == "AUTH_SECRET"
    assert "/arm_1/wrist_camera/image_raw" in gateway["environment"]["MJPEG_SOURCE_URL"]
    assert "qos_profile=sensor_data" in gateway["environment"]["MJPEG_SOURCE_URL"]
    assert "--no-udp" in turn["command"]
