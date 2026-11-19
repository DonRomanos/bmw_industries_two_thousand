# Sourced from ~/.bashrc: pick the best rendering backend the container can see for Gazebo, RViz2 and Vulkan.
# Order: WSL2 GPU (/dev/dxg) > NVIDIA container runtime > DRM render node (AMD, Intel, nouveau) > software.
# Override with KITT_RENDER=software|wsl|nvidia|dri before starting a terminal.

kitt_pick_render() {
    case "${KITT_RENDER:-auto}" in
        software|wsl|nvidia|dri) echo "$KITT_RENDER"; return ;;
    esac
    if [ -e /dev/dxg ]; then echo wsl
    elif [ -e /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && [ -e /dev/nvidiactl ]; then echo nvidia
    elif compgen -G "/dev/dri/renderD*" >/dev/null && [ -r "$(compgen -G '/dev/dri/renderD*' | head -n1)" ]; then echo dri
    else echo software
    fi
}

KITT_RENDER_MODE=$(kitt_pick_render)
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE __GLX_VENDOR_LIBRARY_NAME __EGL_VENDOR_LIBRARY_FILENAMES VK_ICD_FILENAMES

case "$KITT_RENDER_MODE" in
    wsl)
        export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        export GALLIUM_DRIVER=d3d12
        ;;
    nvidia)
        export __GLX_VENDOR_LIBRARY_NAME=nvidia
        export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
        ;;
    dri) ;;
    *)
        export LIBGL_ALWAYS_SOFTWARE=1
        ;;
esac
export KITT_RENDER_MODE
unset -f kitt_pick_render
