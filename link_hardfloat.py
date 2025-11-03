# THIS CODE FORCES PLATFORMIO TO ADD HARDWARE FLOAT FUNCTIONALITY

Import("env")

env.Append(
    CCFLAGS=[
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=softfp",   # <- match platformio.ini
    ],
    LINKFLAGS=[
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=softfp",   # <- match platformio.ini
    ]
)

print("✅ FPU flags applied via pre-script (softfp)")