#!/bin/bash
# -*- mode: shell-script -*-

function rossetaeroarm() {
    rossetmaster sinope 11311
    export ROBOT=AEROARM
    rossetip
}
