set(GUI_INCLUDES
    ${ARCH_INCLUDES}
    )

set(GUI_SOURCES
    render.cpp render_scalers.cpp render_scalers.h
    render_templates.h render_loops.h render_simple.h
    render_templates_sai.h render_templates_hq.h
    render_templates_hq2x.h render_templates_hq3x.h
    )

if (NOT USE_HEADLESS)
    list(APPEND GUI_SOURCES
        sdlmain.cpp sdl_mapper.cpp dosbox_logo.h
        sdl_gui.cpp render_glsl.h
        )
endif ()

add_library(gui STATIC ${GUI_SOURCES})
target_include_directories(gui PRIVATE ${GUI_INCLUDES})