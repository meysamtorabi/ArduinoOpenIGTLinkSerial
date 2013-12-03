
# This file is originally taken from OpenIGTLink examples. 
# Edited by Meysam Torabi, Dec 03 2013 - SNR Lab - BWH.

PROJECT(Receiver)

cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(OpenIGTLink REQUIRED)

include(${OpenIGTLink_USE_FILE})

ADD_EXECUTABLE(ArduinoOpenIGTLinkSerialConnector  ArduinoOpenIGTLinkSerialConnector.cxx)
TARGET_LINK_LIBRARIES(ArduinoOpenIGTLinkSerialConnector OpenIGTLink)

