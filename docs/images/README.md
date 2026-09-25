# Screenshot index

Place the LabVIEW screenshots in this folder with **exactly** these names (the README and
[labview-host.md](../labview-host.md) already reference them):

| Target filename | Which screenshot |
|---|---|
| `labview-front-panel-5ms.png` | Front panel with **clean sine waves** (±230 V, ±10 A), `Sample Period (ms)` = **5**, `Frames Sent` ≈ 2905, `error out` code 0 |
| `labview-front-panel-run.png` | Front panel with the **small growing waveform**, `Sample Period (ms)` = **50**, `Frames Sent` = 124, quality gauge at 0, `Received Frame (Hex)` = `AA02 …` |
| `labview-front-panel-transient.png` | Front panel with the **impulse spikes**, `Sample Period (ms)` = **20**, `Frames Sent` = 55, `Raw PF` = 0,1, full sub-score bars |
| `labview-visa-read.png` | Small screenshot of the **VISA Read** function icon and its terminals (resource in, byte count, read buffer, return count) |
| `labview-block-diagram.png` | The `main.vi` **block diagram** (full window with VISA, Generate_Viv, Build_Frame.vi and the loops) |

Tip: rename copies of the originals (`Capture d'écran …`) to the target names before
uploading — spaces and accents in filenames cause problems in some tools.
