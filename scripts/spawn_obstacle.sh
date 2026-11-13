#!/usr/bin/env bash
# Drop an unmapped box into the running sim to exercise dynamic avoidance. Usage: spawn_obstacle.sh [x] [y]
X=${1:--3.0}; Y=${2:-0.3}
gz service -s /world/office/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req "sdf: '<sdf version=\"1.8\"><model name=\"surprise_box\"><static>true</static><pose>$X $Y 0.4 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.5 0.5 0.8</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.5 0.5 0.8</size></box></geometry><material><ambient>1 0.3 0 1</ambient><diffuse>1 0.3 0 1</diffuse></material></visual></link></model></sdf>'"
