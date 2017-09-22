# nodegcs
MAVLink ground control station on node

## Overview

![](http://7nj0fx.com1.z0.glb.clouddn.com/nodegcs.gif)

nodegcs is a command-line ground control station(GCS). It is based-on nodejs. The intent is for a minimalist and extendable GCS for any UAV supporting the MAVLink protocol (including APM and px4).

nodegcs is capable of connecting a variety of data links including serial and tcp. A lot of node-based application can be extended from nodegcs, such as node-based UAV server(or UAV cloud) or web-based desktop application(Electron or nwjs).



## Usage

To run this example:

```bash
npm install
electron .
```

## 