ifm3d Software Compatibility Matrix
-----------------------------------
<table>
  <tr>
    <th>ifm3d version</th>
    <th>O3D Firmware Version</th>
    <th>O3X Firmware Version</th>
    <th>Ubuntu Linux Version</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>1.6.2114</td>
    <td>0.1.4</td>
    <td>16.04</td>
    <td>Initial (beta) release</td>
  </tr>
  <tr>
    <td>0.2.0</td>
    <td>1.6.2114</td>
    <td>0.1.16, 0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Software triggering (O3X), support for Ubuntu 14.04</td>
  </tr>
  <tr>
    <td>0.3.0</td>
    <td>1.6.2114</td>
    <td>0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Parsing extrinsics (O3D), NTP support (O3X), Simple GUI</td>
  </tr>
  <tr>
    <td>0.3.1</td>
    <td>1.6.2114</td>
    <td>0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Fixed 14.04 regression (std::put_time)</td>
  </tr>
  <tr>
    <td>0.3.2</td>
    <td>1.6.2114</td>
    <td>0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Patch to tools module build script</td>
  </tr>
  <tr>
    <td>0.3.3</td>
    <td>1.6.2114</td>
    <td>0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Windows build support</td>
  </tr>
  <tr>
    <td>0.4.0</td>
    <td>1.6.2114</td>
    <td>0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Added bash completions for ifm3d command line tool</td>
  </tr>
  <tr>
    <td>0.5.0</td>
    <td>1.6.2114</td>
    <td>0.1.20</td>
    <td>14.04, 16.04</td>
    <td>Added firmware flashing to ifm3d command line, Windows support for
    image module</td>
  </tr>
  <tr>
    <td>0.6.0</td>
    <td>1.6.2114</td>
    <td>1.0.62</td>
    <td>14.04, 16.04</td>
    <td>Added pcic client, ability to dump camera trace logs</td>
  </tr>
  <tr>
    <td>0.7.0</td>
    <td>1.6.2114, 1.20.973</td>
    <td>1.0.62</td>
    <td>14.04, 16.04</td>
    <td>Timestamping of image buffers, host/device time sync for O3D, changing
    application parameters on-the-fly</td>
  </tr>
  <tr>
    <td>0.8.1</td>
    <td>1.6.2114, 1.20.973</td>
    <td>1.0.62</td>
    <td>14.04, 16.04</td>
    <td>Register illumination temperature to frame data</td>
  </tr>
  <tr>
    <td>0.8.2</td>
    <td>1.6.2114, 1.20.973</td>
    <td>1.0.62</td>
    <td>14.04, 16.04</td>
    <td>Patches to windows build</td>
  </tr>
  <tr>
    <td>0.9.0</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04</td>
    <td>Dropped support for Ubuntu 14.04, moved to C++14, cmake 3.5, removed
    viewer application, PCL is now optional, provides a framework for
    implementing alternative image containers, ability to set password on
    camera.</td>
  </tr>
  <tr>
    <td>0.9.1</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04</td>
    <td>Removed some additional Boost dependencies</td>
  </tr>
  <tr>
    <td>0.9.2</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04, 18.04</td>
    <td>Added support for Ubuntu 18.04</td>
  </tr>
  <tr>
    <td>0.9.3</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>Docs and minor patches</td>
  </tr>
  <tr>
    <td>0.10.0</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>
      Session IDs can be explicitly set and shared across camera
      instances. Speed improvements to JSON imports.
    </td>
  </tr>
  <tr>
    <td>0.11.0</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>Added a jitter subcommand to ifm3d, camera intrinsics exposed</td>
  </tr>
  <tr>
    <td>0.11.1</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>Bad pixels are now flagged by the driver, opencv image container module
    headers can be included in multiple translation units, and other minor bug
    fixes.</td>
  </tr>
  <tr>
    <td>0.11.2</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>Less verbose logging in framegrabber for better memory, disk, cpu
    consumption on long-running embedded systems</td>
  </tr>
</table>
