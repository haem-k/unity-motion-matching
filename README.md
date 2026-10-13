# Simple Motion Matching in Unity

A Unity example of feature-based motion matching for character locomotion. The project includes motion playback, root-space pose manipulation, feature visualization, displacement-based transition blending, and KD-tree nearest-neighbor search.

**Character models and motion data are not included.** The example scenes preserve the original setup, but require local assets and manual reference reassignment before playback.

## Requirements

- Unity **2022.3.62f3**
- A rigged character and compatible animation clips, obtained separately.
- [KdTree](https://github.com/codeandcats/KdTree), installed separately as described below.

### Installing KdTree

Download the [KdTree repository](https://github.com/codeandcats/KdTree) and copy its `KdTreeLib` folder, including the `Math` subfolder, into `Assets/MotionMatching/Scripts/External/`. Create the `External` folder if necessary. Keep a copy of the upstream `LICENSE` inside the local `KdTreeLib` folder.

The library source and its Unity metadata are excluded from this repository and ignored by Git. Install the library before running the project; `MatchingDatabase.cs` depends on it.

## External assets

| Resource | Source | Local use |
| --- | --- | --- |
| Y Bot character | [Mixamo](https://www.mixamo.com/) | Download the rigged character and import it locally. |
| LAFAN1 motion data | [Ubisoft La Forge Animation Dataset](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) | Download the dataset and prepare animation clips compatible with your character. |

Please follow each provider's terms. This project uses LAFAN1 walk, run, and sprint sequences mirrored and retargeted to the Y Bot character.

### Exporting motion assets

`MotionExporter` samples animation clips into a `Motion` ScriptableObject containing the projected ground root, joint-local and joint-global matrices, and parent indices for each frame.

1. Create a temporary export scene and place the configured character in it.
2. Create a separate GameObject and add `MotionExporter`.
3. Assign its `character` and `animClips` fields. Clips must animate the actual transforms of this character; verify sampling after retargeting.
4. Enter Play mode in the Editor. `Start()` exports each clip to `Assets/MotionMatching/Data/<clip.name>.asset`.
5. Exit Play mode and disable or remove the exporter before normal playback.

## Example scenes

Scenes are in `Assets/MotionMatching/Scenes/`.

| Scene | Purpose |
| --- | --- |
| `0_MotionPlayTest.unity` | Play a generated motion asset. |
| `1_PoseTest.unity` | Inspect root-space pose alignment. |
| `2_FeatureTest.unity` | Visualize pose and future-trajectory features. |
| `3_BlendingTest.unity` | Demonstrate displacement-based transition blending. |
| `4_MotionMatching.unity` | Control locomotion through motion matching. |

After restoring assets, start with motion playback and then open `4_MotionMatching.unity`. Use the arrow keys through the default Horizontal/Vertical input axes.

1_PoseTest Demo

https://github.com/user-attachments/assets/9c3234c7-cf82-43a6-a667-c1304dc59e09


2_FeatureTest Demo

https://github.com/user-attachments/assets/4aadeeb8-3a13-47fb-b38b-141524ea275a


3_BlendingTest Demo

https://github.com/user-attachments/assets/c6659dd2-895b-49a3-8c05-837c3bd33b2d


4_MotionMatching Demo

https://github.com/user-attachments/assets/457608c0-05fd-4849-9bc0-ec1897ef40a4


## Limitations

- Example scenes require external data and manual setup.
- The original transition fallback assumes sufficient preceding frames and can index out of range for some states.

## Acknowledgements

This project uses [KdTree](https://github.com/codeandcats/KdTree) by codeandcats for nearest-neighbor search. The library is distributed under the [MIT License](https://github.com/codeandcats/KdTree/blob/master/LICENSE) and must be obtained separately.
