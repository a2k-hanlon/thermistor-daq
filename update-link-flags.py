Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#

env.Append(
    # Add linker flags to enable use of floating point hardware
    LINKFLAGS=[
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard"
    ]
)
