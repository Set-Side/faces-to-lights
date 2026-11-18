# Set-Side Faces to Lights

Turn selected mesh faces into matching area lights in Blender: same position, direction and size, with the faces glowing to match. Made for windows, light panels, screens, signs and any lit surface you've modelled.

**[Get it free on Blender Extensions](https://extensions.blender.org/add-ons/setside-faces-to-lights/)**

Free and open source (GPL-3.0-or-later). Works with Blender 4.2 LTS and newer, in Cycles and EEVEE.

## Install

- **In Blender:** Edit > Preferences > Get Extensions, search for "Faces to Lights", click Install.
- **From a file:** download the zip from its [Blender Extensions page](https://extensions.blender.org/add-ons/setside-faces-to-lights/), then drag it into Blender, or use Preferences > Add-ons > Install from Disk.

## Quick start

1. Select a mesh and press Tab for Edit Mode.
2. Select the faces you want to turn into lights.
3. Open the sidebar (N) > **Set-Side** tab > **Faces to Lights**, or Face menu (Ctrl+F) > **Faces to Lights**.
4. Adjust the options in the panel that appears at the bottom left of the viewport.

Later, select the mesh (or any of its lights) and use **Edit Lights** in the Set-Side tab to change all its lights at once.

## Features

| Feature | What it does |
| --- | --- |
| Lights that fit the faces | Each face, or group of similar faces, gets an area light matched to its real area. On curved surfaces lights sit just in front, so they don't overlap. |
| Merge Similar Faces | Neighbouring faces facing the same way (within the angle you set) share one light, so a panel made of many faces is one light. |
| Match Surface / Cover Outline | Size lights by the faces' real area, or to cover their whole outline. |
| Make Faces Glow | The faces glow in the light's colour, so they look lit on camera without adding render noise. |
| Edit Lights | Power, colour or colour temperature (kelvin), spread, glow brightness and visibility change the lights and glowing faces together, live. Each mesh remembers its own settings. |
| Power per m² | Big and small windows look consistent. Uses real metres, whatever the scene's Unit Scale. |
| Update Lights | After moving, scaling or editing the mesh, re-fits every light to its faces. |
| Remove Lights | Deletes a mesh's lights and gives its faces their original materials back. |
| Don't Light Source Mesh | Uses Blender's light linking so a mesh isn't lit by its own lights. |
| Light Portal | Makes the lights Cycles light portals, for daylight through windows. |
| Visible to Camera / in Reflections | Show or hide the light shapes themselves in renders. |
| Light Shapes: Shown / Hidden | One button hides light outlines in the viewport, keeping their light. |
| Delete Leftover Lights | If you delete a mesh, its lights can be cleaned up in one click. |

## Tips

- **Too many lights?** Turn on Merge Similar Faces and raise the angle. Hundreds of lights slow Blender down.
- **Colours look washed out?** Blender's AgX view turns very bright colours white. Lower the power or glow, or lower Film Exposure, rather than raising them.
- **Running it again on the same faces** replaces their lights rather than adding more.
- **Scenes in centimetres** (Unit Scale 0.01) are handled: power per m² is in real square metres.

## Problems or ideas

Please open an issue on this repository with your Blender version, what you did and what happened. A screenshot helps.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE). © 2026 Set-Side.
