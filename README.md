# Nodegcs
MAVLink ground control station on node

## Overview

![](http://7nj0fx.com1.z0.glb.clouddn.com/nodegcs.gif)

Nodegcs is a command-line ground control station(GCS). It is based-on nodejs. The intent is for a minimalist and extendable GCS for any UAV supporting the MAVLink protocol (including APM and px4).

Nodegcs is capable of connecting a variety of data links including serial and tcp. A lot of node-based application can be extended from nodegcs, such as node-based UAV server(or UAV cloud) or web-based desktop application(Electron or nwjs).

Features of nodegcs include:

- It is a console based app, and easy to use.
- It is portable; it may run on Mac/Linux/Win.
- The light-weight design means it can run on embedded device.
- It can be used to build an node-based UAV proxy server.
- Desktop application can be extended using Electron or nwjs.

## Usage

To run this example:

```bash
npm install
node nodegcs
```

### Connect via serial

In nodegcs:

```bash
start /dev/cu.usbserial-A503UTCT 115200
```

### Connect via tcp

You may need to start a simulator first. I recommand dronekit-sitl.

To install & run dronekit-sitl:


```bash
pip install dronekit-sitl
dronekit-sitl copter-3.3 --home=39.9794467,116.3396286,20,353
```

In nodegcs:
```bash
start
```

## License
Nodegcs is released under the GNU General Public License v3 or later.