@echo off
call mogrify -format jpeg *.ppm
call del *.ppm
