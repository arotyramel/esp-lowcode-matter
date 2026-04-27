#!/usr/bin/env python3
import json, copy, os

BASE = "/workspaces/esp-lowcode-matter/products"
OUT_DIR = f"{BASE}/garage_controller/configuration"
os.makedirs(OUT_DIR, exist_ok=True)

def load(product, variant):
    path = f"{BASE}/{product}/configuration/data_model_{variant}.zap"
    with open(path) as f:
        return json.load(f)

def build_zap(variant):
    occ  = load("occupancy_sensor", variant)
    sock2 = load("socket_2_channel", variant)

    # EP1: root — from occupancy_sensor
    root_ep_type = copy.deepcopy(next(e for e in occ["endpointTypes"] if e["id"] == 1))

    # EP2: LIDAR occupancy sensor — from occupancy_sensor EP2
    lidar_ep_type = copy.deepcopy(next(e for e in occ["endpointTypes"] if e["id"] == 2))
    lidar_ep_type["id"] = 2

    # EP3: Gate occupancy sensor — duplicate of EP2
    gate_ep_type = copy.deepcopy(lidar_ep_type)
    gate_ep_type["id"] = 3

    # EP4: Relay On/Off — from socket_2_channel EP2 (fully compliant, no warnings)
    relay_ep_type = copy.deepcopy(next(e for e in sock2["endpointTypes"] if e["id"] == 2))
    relay_ep_type["id"] = 4

    # endpoints array
    root_endpoint  = copy.deepcopy(occ["endpoints"][0])

    lidar_endpoint = copy.deepcopy(occ["endpoints"][1])
    lidar_endpoint["endpointId"] = 2
    lidar_endpoint["endpointTypeRef"] = 2

    gate_endpoint = copy.deepcopy(occ["endpoints"][1])
    gate_endpoint["endpointId"] = 3
    gate_endpoint["endpointTypeRef"] = 3
    gate_endpoint["endpointTypeIndex"] = 2  # index into endpointTypes array: [root, lidar, gate, relay]

    relay_endpoint = copy.deepcopy(sock2["endpoints"][1])
    relay_endpoint["endpointId"] = 4
    relay_endpoint["endpointTypeRef"] = 4
    relay_endpoint["endpointTypeIndex"] = 3  # index into endpointTypes array: [root, lidar, gate, relay]

    return {
        "fileFormat":    occ["fileFormat"],
        "featureLevel":  occ["featureLevel"],
        "creator":       occ["creator"],
        "keyValuePairs": occ["keyValuePairs"],
        "package":       occ["package"],
        "endpointTypes": [root_ep_type, lidar_ep_type, gate_ep_type, relay_ep_type],
        "endpoints":     [root_endpoint, lidar_endpoint, gate_endpoint, relay_endpoint],
    }

for variant in ["thread", "wifi"]:
    zap = build_zap(variant)
    out_path = f"{OUT_DIR}/data_model_{variant}.zap"
    with open(out_path, "w") as f:
        json.dump(zap, f, indent=2)
    print(f"Written: {out_path}  ({os.path.getsize(out_path)} bytes)")

print("Done. Run 'Upload Configuration' then 'Upload Code'.")
