add_executable(testdosbox)
target_sources(testdosbox PUBLIC
        main.m
        )
target_include_directories(testdosbox PRIVATE
        ../public
        )
target_link_libraries(testdosbox oedosbox)
target_compile_options(testdosbox PUBLIC "-fobjc-arc")