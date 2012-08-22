add_executable(AffineRegistration AffineRegistration.cxx)
target_link_libraries(AffineRegistration ${ITK_LIBRARIES})

add_executable(ConvertFromDICOM ConvertFromDICOM.cxx)
target_link_libraries(ConvertFromDICOM ${ITK_LIBRARIES})
