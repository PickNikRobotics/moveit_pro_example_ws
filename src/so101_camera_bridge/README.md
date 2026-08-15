# SO-101 camera bridge

This package reads the Pi's existing MediaMTX streams and republishes them as
ROS 2 images. It never opens `/dev/video*`, changes a Pi service, or sends a
robot command.

| Physical stream | ROS image topic | Frame |
| --- | --- | --- |
| `head` | `/so101/cameras/overhead/image_raw` | `overhead_cam_optical_frame` |
| `gripper` | `/so101/cameras/wrist/image_raw` | `right_wrist_cam_optical_frame` |

The topic contract is identical to the MuJoCo configuration. Override
`SO101_HEAD_RTSP_URL` and `SO101_GRIPPER_RTSP_URL` if the Pi address changes.
The generated `CameraInfo` is an approximate pinhole model until a physical
camera calibration is available.

## Decoding is on demand

The image topics are advertised from startup, so the camera panes list them and
`CameraInfo` is always available. Decoding, however, only runs while something is
actually subscribed to an image topic, and stops again when the last subscriber
leaves.

This matters because the operator's video no longer comes through here. MoveIt Pro
relays the Pi's stream straight to the browser (`MOVEIT_WEBRTC_PASSTHROUGH`), so
decoding every frame to raw and republishing it produced images that nothing read
— measured at 198 MB and 34 threads for two cameras. Recording, perception
Behaviors, and anything else that genuinely wants the pixels still get them by
subscribing.

### Cold-start budget

Subscribing does not produce a frame immediately. Measured against
`rtsp://so101-pi.tail337068.ts.net:8554/head`:

| Step | Measured |
| --- | --- |
| Subscriber matched, `get_subscription_count()` goes non-zero | ~0.14 s |
| Demand check notices (one publish tick at `publish_fps: 15.0`) | ≤0.07 s |
| RTSP connect and first decoded frame | 1.4–2.0 s |

So roughly 1.6–2.2 s from subscribe to first image, against `ExecutePolicy`'s
5 s first-frame timeout. Anything that needs a frame sooner — or that cannot
tolerate the first attempt failing and retrying after `reconnect_delay_seconds`
— should keep a subscriber attached rather than subscribing on demand.

### Not stopping on the first idle tick

`decoder_linger_seconds` (default 5.0) is how long the image topic must stay
unsubscribed before the decoder is torn down. A subscriber that flips away and
back — switching camera panes, one Behavior handing off to the next — reads as
zero subscribers for a fraction of a second; measured pane flipping produced 7
such gaps in 3.2 s. Without the hold-off each gap costs a full RTSP teardown and
a 1.4–2.0 s reconnect on the upstream camera. Set it to 0.0 to stop on the first
idle tick.

### CameraInfo geometry

`image_width` and `image_height` (default 640x480, which matches the Pi's
streams today) describe the camera before anything has been decoded, so
`CameraInfo` does not need a frame. Once the decoder sees a real frame,
`CameraInfo` follows the decoded size instead and the node warns once per
camera: intrinsics that describe a different image than the one published
alongside them silently corrupt every consumer that projects pixels to poses.
Set the parameters to match the stream so the info topic is right during the
window before the first frame.
