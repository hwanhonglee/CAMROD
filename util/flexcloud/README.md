# Georeferenced FlexCloud Artifacts

<!-- HH_260810 - Record imported binary PCD identity without inferring field
accuracy or changing the existing conversion and trajectory utilities. -->

These files are imported georeferenced point-cloud products. They use PCD
`VERSION 0.7`, binary `x/y/z/intensity` float fields, and are stored with Git
LFS. The repository records file identity only; it does not claim surveyed
accuracy or regenerate the clouds.

| File | Points | Bytes | SHA-256 |
|---|---:|---:|---|
| `georef_glim_worak_v2_binary.pcd` | `6,940,175` | `111,042,963` | `89ee95c2052230387eed5154aea6e55dee1918bf2a9a27fff3c01cfcf7045182` |
| `georef_map_local_/georef_map_local_intensity_0.10m_binary.pcd` | `1,766,652` | `28,266,595` | `78358e25c7260e572706b014de5f3d3995f541576fc89c1e74770e0b8cda34f8` |
| `georef_map_local_/georef_map_local_lane_i300_0.03m_binary.pcd` | `151,768` | `2,428,449` | `65ba7e486695e49083a798f7750bc90fc80cc62ed078a13c0bd705fdeef41f72` |

The existing ZIP archives remain unchanged. Lanelet map snapshots remain at
the repository root and are versioned as ordinary text files.
