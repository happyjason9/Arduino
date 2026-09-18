# PlatformIO 建置腳本 (extra_scripts)
#
# 為什麼需要這個檔案：
# msys64 的 GNU ld 2.45 在連結 Unity 測試程式時會失敗 (exit 1) 且不印任何診斷訊息。
# 同一組 .o 檔改用 LLVM lld 連結則完全正常。
#
# 注意 build_flags 只會傳給「編譯」步驟，不會進到「連結」命令列，
# 所以 -fuse-ld=lld 必須透過 LINKFLAGS 注入。
Import("env")
env.Append(LINKFLAGS=["-fuse-ld=lld"])
