project(ToFCrustFunctionalTests)

add_subdirectory (ib-test)
add_subdirectory (sequ-ctrl)
add_subdirectory (set-lens-info)
add_subdirectory (tof-rw)
add_subdirectory (tof-log)
add_subdirectory (tof-sdram-test)

install(TARGETS ib-test sequ-ctrl set-lens-info tof-rw tof-log tof-sdram-test
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
