# Fire Detection System

<a href="https://github.com/nmpluta/fire-detection-system/actions/workflows/build-using-docker.yml?query=branch%3Amain">
  <img src="https://github.com/nmpluta/fire-detection-system/actions/workflows/build-using-docker.yml/badge.svg?event=push">
</a>
<a href="https://github.com/nmpluta/fire-detection-system/actions/workflows/docs.yml?query=branch%3Amain">
  <img src="https://github.com/nmpluta/fire-detection-system/actions/workflows/docs.yml/badge.svg?event=push">
</a>
<a href="https://nmpluta.github.io/fire-detection-system">
  <img alt="Documentation" src="https://img.shields.io/badge/documentation-3D578C?logo=sphinx&logoColor=white">
</a>
<a href="https://nmpluta.github.io/fire-detection-system/doxygen">
  <img alt="API Documentation" src="https://img.shields.io/badge/API-documentation-3D578C?logo=c&logoColor=white">
</a>

This repository contains an nRF Connect SDK based Fire Detection System application. This project is designed for early detection of wildfires using a network of sensors monitoring temperature, humidity, and smoke levels. The system is built on the Zephyr RTOS and utilizes LTE-M communication for reliable data transmission even in remote areas.

## Getting started

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Installation guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation/install_ncs.html).

### Initialization

The first step is to initialize the workspace folder (``fire-detection-system-workspace``) where
the ``fire-detection-system`` and all nRF Connect SDK modules will be cloned. Run the following
command:

```shell
# initialize fire-detection-system-workspace for the fire-detection-system (main branch)
west init -m https://github.com/nmpluta/fire-detection-system --mr main fire-detection-system-workspace
# update nRF Connect SDK modules
cd fire-detection-system-workspace
west update
```

### Building and running

To build the application, run the following command:

```shell
cd fire-detection-system
west build -b $BOARD app
```

where `$BOARD` is the target board.

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build -b $BOARD app -- -DEXTRA_CONF_FILE=debug.conf
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```

### Testing

To execute Twister integration tests, run the following command:

```shell
west twister -T tests --integration
```

### Documentation

A minimal documentation setup is provided for Doxygen and Sphinx. To build the
documentation first change to the ``doc`` folder:

```shell
cd doc
```

Before continuing, check if you have Doxygen installed. It is recommended to
use the same Doxygen version used in [CI](.github/workflows/docs.yml). To
install Sphinx, make sure you have a Python installation in place and run:

```shell
pip install -r requirements.txt
```

API documentation (Doxygen) can be built using the following command:

```shell
doxygen
```

The output will be stored in the ``_build_doxygen`` folder. Similarly, the
Sphinx documentation (HTML) can be built using the following command:

```shell
make html
```

The output will be stored in the ``_build_sphinx`` folder. You may check for
other output formats other than HTML by running ``make help``.
