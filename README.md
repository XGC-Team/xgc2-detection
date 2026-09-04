# xgc2-detection

Public XGC-Team ROS 1 Noetic detection product.

The imported source trees still live under the historical `temp/` directory,
but **that directory name no longer means parked or reference-only**. Current CI
copies these trees into an isolated catkin workspace, builds amd64/arm64 Debian
packages, creates trusted build manifests, and the release workflow publishes
the product through the central XGC2 release train.

| Path | Origin | Current role |
| --- | --- | --- |
| `temp/onboard_detector_lv` | LV-DOT/onboard_detector_lv | packaged source |
| `temp/fapp_obj_state_msgs` | FAPP object-state messages | packaged source |
| `temp/fapp_mot_mapping` | FAPP mapping/tracking | packaged source |

The canonical product metadata is `.xgc2/product.yml`. Packaging logic under
`.xgc2/scripts/` is authoritative for which source trees enter the Debian
artifacts; do not infer lifecycle from the legacy `temp/` path name.

Clone:

```bash
git clone git@github.com:XGC-Team/xgc2-detection.git
```
