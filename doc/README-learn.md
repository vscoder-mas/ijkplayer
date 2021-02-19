### ijkplayer学习笔记
1. ffpipenode_android_mediacodec_vdec.c  硬编码创建相关.c文件
    + pipenode -> func_run_sync 创建线程 enqueue avpacket

2. ffplay Frame 关系
    + FrameQueue -> Frame[] -> PacketQueue -> MyAVPacketList -> AVPacket
3. VideoState->auddec(Decoder)->queue = VideoState->audioq(PacketQueue)

