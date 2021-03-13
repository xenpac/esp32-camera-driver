# not necessary to specify sources. by default it will take all c-files in main-directory.
#set(COMPONENT_SRCS "camdrv.c" "i2sdrv.c" "jpgcam2640.c" "tcpserver.c" "ov2640.c" INCLUDE_DIRS ".include")

set(COMPONENT_EMBED_FILES "index_ov2640.html.gz")

register_component()
