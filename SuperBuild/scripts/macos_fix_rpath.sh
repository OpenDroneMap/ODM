#!/bin/bash

install_name_tool -add_rpath "$1" "$2" || true
