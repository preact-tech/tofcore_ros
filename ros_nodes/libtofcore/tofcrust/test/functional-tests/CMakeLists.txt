project(ToFCrustFunctionalTests)

add_subdirectory (ib-test)
add_subdirectory (sequ-ctrl)
add_subdirectory (set-lens-info)
add_subdirectory (tof-rw)
add_subdirectory (tof-limits)
add_subdirectory (tof-log)
add_subdirectory (tof-sdram-test)
add_subdirectory (tof-therm)

install(TARGETS ib-test sequ-ctrl set-lens-info tof-rw tof-limits tof-log tof-sdram-test tof-therm
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
