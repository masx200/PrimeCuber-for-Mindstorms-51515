LEGO Mindstorms 51515 的 PrimeCuber 变体

该代码是 David Gilday （mindcuber.com） 的作品。这只是一个 Mindstorms 51515 端口。

构建改编是由我的儿子 Marin 使用 Primecuber 说明完成的，并且只有 Mindstorms 51515 部件。

Orione（http://www.orione-alnitak.eu/responsive）非常喜欢这个设计，以至于他（她？他们？）做了说明：

https://drive.google.com/file/d/1bPBpQtQXGo1PtdjhdGkOFznhE9t5BwvA/view

谢谢Orione！

求解器可以在 https://youtu.be/e2EDLIPwlSM 上看到

注意：

用于将程序加载到 Mindstorms 集线器的原始方法不适用于最新固件。
这是取自 MindCuber-RI 的一种新方法。无论如何，整件事都是由大卫·吉尔戴（David Gilday）写的，所有的功劳都归功于他。

我喜欢这个 PrimeCuber 改编版，因为它是由我 10 岁的儿子 Marin 完成的，但 MindCuber-RI 是我最喜欢的乐高建筑之一，还有乐高机械组 42006 挖掘机和乐高机械组反铲装载机 8069（出于情感原因）。:)

指示

1. 在未使用的插槽（例如 18）中打开并运行程序“PCRISolver-v1p4.lms”

一分钟后，将出现一条错误消息：

SyntaxError：语法无效

这是意料之中的！

2. 在另一个未使用的插槽（例如 19）中打开并运行程序“PCRIInstall-v1p4.lms”

需要等待 1-2 分钟才能看到转盘（某种程度上）在集线器上移动。
预期输出为：

>插槽： 18 /projects/420/__init__.py 71645B
        > 安装中...
        节省>： /pccolors_vlp4.py 7706B
        节省>： /pcsolver_vlp4.py 22189B
        节省>： /primecuber_v1p4.py 14121B
        节省>： /pcmtab1_v1p4.bin 18985B
        > 已安装 PrimeCuber-RI v1p4
        
在此之后，您可以删除插槽 18 和 19 中的程序
        
3. 打开并运行程序“PrimeCuber-RI-v1p4.lms”

此程序调用以前加载的程序并求解多维数据集。
在插入立方体之前，可以使用左右按钮来正确对齐支架。

文件说明

PCRISolver-v1p4.lms

包含预期和故意语法错误的文件。包含下一个程序将加载到中心的程序的文本。

PCRIInstall-v1p4.lms

加载器，将.py文件加载到头脑风暴中心。与原来的PrimeCuber相比，修改是在ScanFace功能中（因为扫描臂设计不同）和代码的开头，因此每次都会安装程序文件，即使它们存在于集线器中。

PrimeCuber-RI-v1p4.lms

通过调用 PCRIInstall-v1p4.lms 加载的 .py 文件中的函数来解决多维数据集的项目。
首次启动后，不再需要连接到 PC。

立方体测试扫描 1.lms

一个快速而肮脏的程序，用于确定正确的扫描臂位置：

- 停车位置
- 角方块
- 侧方块
- 中间广场

PrimeCuber-RI-Clean-Up.lms

一个小程序，用于从集线器中删除PrimeCuber-RI文件。

MindCuber-RI-Clean-Up.lms

一个小程序，它从集线器中删除MindCuber-RI文件。

-------------------------------------------------------------------------------------

与原始 David Gilday 的代码（请在 mindcuber.com 查看）相比，唯一的变化是在“PCSolver-v1p4”模块中。

以下是更改的部分（在ScanFace功能中）：

#### # MD：中间：是 +485
#### self.run_to（self.motor_scan， self.motor_scan_base+195， 100）
#### 自我。扫描RGB（f， 8）
#### self.motor_tilt.brake（）
#### 如果 self.slower：
#### self.slower = 假
#### self.scan_speed -= 1
#### # MD：已取消注释：
#### print（“扫描速度 ”+str（self.scan_speed））
#### self.run_nw（self.motor_turn， self.motor_turn_base+self.turn_ratio*360， self.scan_speed）
#### for i in range（4）：
#### # MD： 角： was 300
#### 自我。扫描件（145， f， o， i）
#### # MD： side： was 365
#### 自我。扫描件（165， f， o+1， i+4）

此外，在视频中，电机和传感器连接到不同的端口。如果遵循原始 PrimeCuber 说明，则无需进行此更改。

####class primecuber（）：
...
#### self.sensor_color = self.check_port（hub.port.B， False， [61]， 4， 0）
#### self.sensor_dist= self.check_port（hub.port.C， False， [62]， 0， 2）
#### self.motor_scan= self.check_port（hub.port.D， true，[48， 75]， 4， 2）
#### self.motor_turn= self.check_port（hub.port.F， true，[48， 75]， 4， 4）
#### self.motor_tilt= self.check_port（hub.port.E， true，[48， 75]， 0， 4）

求解程序启动后，如果未找到预期的电机/传感器，则违规端口附近的像素将亮起。

最后，在同一模块的开头（第 54 行），插入了以下代码，以便将程序重新安装到集线器中，即使它已经存在：

# MD：强制重装
installed = 假

享受！

姆拉登