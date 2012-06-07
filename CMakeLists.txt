cmake_minimum_required(VERSION 2.8)
set(CMAKE_INSTALL_RPATH "\\$ORIGIN/../..")

# configuration
set(TARGET_PATH	"${CMAKE_SOURCE_DIR}/../tricore/" CACHE PATH "tricore GLISS2 directory")
set(GLISS_PATH	"${CMAKE_SOURCE_DIR}/../gliss2" CACHE PATH "GLISS source directory")

# otawa dependency
find_program(OTAWA_CONFIG otawa-config DOC "OTAWA configuration file")
if(NOT OTAWA_CONFIG)
	message(FATAL_ERROR "FATAL: OTAWA not found but required !")
endif()
execute_process(COMMAND otawa-config --cflags OUTPUT_VARIABLE OTAWA_CXXFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND otawa-config --libs OUTPUT_VARIABLE OTAWA_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND otawa-config --prefix OUTPUT_VARIABLE OTAWA_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)

# internals
set(ARCH			"tricore")
set(TARGET			"tricore")
set(ELF_NUM			"44")
set(TARGET_LIB		"${TARGET_PATH}/src/lib${TARGET}.a")
set(TARGET_INC		"${TARGET_PATH}/include")
set(TARGET_IRG		"${TARGET_PATH}/${TARGET}.irg")
set(GLISS_ATTR		"${GLISS_PATH}/gep/gliss-attr")
set(OTAWA_KIND 		"${PROJECT_BINARY_DIR}/kind.h")
set(OTAWA_TARGET 	"${PROJECT_BINARY_DIR}/target.h")
set(OTAWA_SEM		"${PROJECT_BINARY_DIR}/sem.h")
message(STATUS "GLISS_ATTR = ${GLISS_ATTR}")

set(SOURCES
	"${ARCH}.cpp"
	"${OTAWA_KIND}"
	"${OTAWA_TARGET}"
	"${OTAWA_SEM}")
#)

# build of GLISS-derived sources
if(NOT MINGW_WIN)
	add_custom_command(
		OUTPUT ${OTAWA_KIND} DEPENDS "kind.tpl" "kind.nmp" COMMAND ${GLISS_ATTR}
		ARGS ${TARGET_IRG} -o ${OTAWA_KIND} -a otawa_kind -f -t "kind.tpl" -d "return 0\\;" -e "kind.nmp"
		DEPENDS ${TARGET_IRG}
	)
	add_custom_command(
		OUTPUT ${OTAWA_TARGET} DEPENDS "target.tpl" "target.nmp" COMMAND ${GLISS_ATTR}
		ARGS ${TARGET_IRG} -o ${OTAWA_TARGET} -a otawa_target -f -t "target.tpl" -d "return 0\\;" -e target.nmp
		DEPENDS ${TARGET_IRG}
	)
	add_custom_command(
		OUTPUT ${OTAWA_SEM} DEPENDS "sem.tpl" "sem.nmp" COMMAND ${GLISS_ATTR}
		ARGS ${TARGET_IRG} -o ${OTAWA_SEM} -p -a sem -t "sem.tpl" -d "\\;" -e sem.nmp
		DEPENDS ${TARGET_IRG}
	)
endif()

# build the library
include_directories("${TARGET_INC}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OTAWA_CXXFLAGS}")
#set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} ${OTAWA_LDFLAGS}")
message(STATUS "OTAWA_CXXFLAGS=${OTAWA_CXXFLAGS}")
add_library(${ARCH} SHARED ${SOURCES})
set_target_properties(${ARCH} PROPERTIES
	COMPILE_FLAGS "${OTAWA_CXXFLAGS}"
	LINK_FLAGS "${OTAWA_LDFLAGS}")
set_property(TARGET ${ARCH} PROPERTY PREFIX "")
set(SUFFIX "${CMAKE_SHARED_LIBRARY_SUFFIX}")
target_link_libraries(${ARCH} "${TARGET_LIB}" gel)

# installation
set(INSTALL_PATH ${OTAWA_PREFIX}/lib/otawa/loader)
install(TARGETS ${ARCH} DESTINATION ${INSTALL_PATH})
if(WIN32)
	file(WRITE elf_${ELF_NUM}.link "${ARCH}.dll")
	install(FILES elf_${ELF_NUM}.link DESTINATION ${INSTALL_PATH})
elseif(UNIX OR APPLE)
	add_custom_command(DEPENDS ${ARCH} OUTPUT "elf_${ELF_NUM}${SUFFIX}" COMMAND ln -s "${ARCH}${SUFFIX}" "elf_${ELF_NUM}${SUFFIX}")
	add_custom_target(ELF_LINK ALL DEPENDS "elf_${ELF_NUM}${SUFFIX}")
	install(FILES "elf_${ELF_NUM}${SUFFIX}" DESTINATION ${INSTALL_PATH})
endif()


