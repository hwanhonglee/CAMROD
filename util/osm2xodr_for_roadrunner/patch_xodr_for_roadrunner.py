import re
import sys
import xml.etree.ElementTree as ET

try:
    from pyproj import Transformer
except ImportError:
    print("pyproj is required.")
    print("Install with: pip install pyproj")
    sys.exit(1)


if len(sys.argv) != 3:
    print("Usage:")
    print("  python3 patch_xodr_for_roadrunner.py input.xodr output.xodr")
    sys.exit(1)


input_path = sys.argv[1]
output_path = sys.argv[2]

tree = ET.parse(input_path)
root = tree.getroot()

header = root.find("header")
if header is None:
    raise RuntimeError("No <header> found in XODR file.")

geo_ref = header.find("geoReference")
offset = header.find("offset")

if offset is None:
    raise RuntimeError("No <offset> found in XODR header.")

offset_x = float(offset.attrib.get("x", "0"))
offset_y = float(offset.attrib.get("y", "0"))

# SUMO usually writes offset as negative UTM origin.
utm_origin_x = -offset_x
utm_origin_y = -offset_y

print(f"Detected SUMO offset x/y: {offset_x}, {offset_y}")
print(f"Estimated UTM origin x/y: {utm_origin_x}, {utm_origin_y}")

# Default for Korea map area: UTM Zone 52N, EPSG:32652
transformer = Transformer.from_crs("EPSG:32652", "EPSG:4326", always_xy=True)
origin_lon, origin_lat = transformer.transform(utm_origin_x, utm_origin_y)

print(f"Estimated origin lat/lon:")
print(f"  lat = {origin_lat}")
print(f"  lon = {origin_lon}")

# Remove old geoReference if exists
if geo_ref is not None:
    header.remove(geo_ref)

# Add local Transverse Mercator georeference
new_geo_ref = ET.Element("geoReference")
new_geo_ref.text = (
    f"<![CDATA[+proj=tmerc +lat_0={origin_lat:.10f} +lon_0={origin_lon:.10f} "
    f"+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs]]>"
)

# xml.etree escapes CDATA markers, so we will fix this after writing.
header.insert(0, new_geo_ref)

# Set offset to zero
offset.set("x", "0.00")
offset.set("y", "0.00")
offset.set("z", "0.00")
offset.set("hdg", "0.00")

tree.write(output_path, encoding="UTF-8", xml_declaration=True)

# Fix escaped CDATA
with open(output_path, "r", encoding="UTF-8") as f:
    text = f.read()

text = text.replace(
    "&lt;![CDATA[",
    "<![CDATA["
).replace(
    "]]&gt;",
    "]]>"
)

# Pretty basic cleanup
with open(output_path, "w", encoding="UTF-8") as f:
    f.write(text)

print(f"Written patched XODR: {output_path}")
