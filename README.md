# ir_decoder_base_sigrokdecode
在写一个远程红外遥控的时候遇到的问题。接上红外感应后，逻分上看到的是丢失载波的信号，这通常不能被常见的解析器解析。于是我修改了sigrok中的RGB灯解析器，使其能够解析TCL空调信号。
思路是基于占空比时间，来鉴别起始信号，逻辑1，逻辑0，结束信号
