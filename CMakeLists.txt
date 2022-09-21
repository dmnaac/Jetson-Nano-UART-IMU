cmake_minimum_required(VERSION 2.8)

project(jetson_uart)

add_executable(jetson_uart jetson_uart.cpp JY901.c)
