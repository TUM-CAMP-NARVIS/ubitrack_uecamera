from conans import ConanFile


class UbitrackConan(ConanFile):
    name = "ubitrack"
    version = "1.3.0"
    settings = "os", "arch", "compiler", "build_type"
    short_paths = True
    url = "https://github.com/Ubitrack/ubitrack.git"
    description = "Meta-Package for TUM CAMPAR Ubitrack Tracking and Sensor Fusion Framework"
    license = "GPL"
    generators = "ubitrack_virtualenv_generator"



    def configure(self):
        if self.settings.os == "Linux":
            self.options["opencv"].with_gtk = True

    def requirements(self):
        userChannel = "ubitrack/stable"



        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_component_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel) )
        self.requires("ubitrack_facade/%s@%s" % (self.version, userChannel) )
        self.requires("ubitrack_virtualenv_generator/[>=%s]@ubitrack/stable" % self.version)

        self.requires("ubitrack_vision/[>=%s]@%s" % (self.version, userChannel)) 
        self.requires("ubitrack_component_vision/[>=%s]@%s" % (self.version, userChannel))
        self.requires("ubitrack_component_visualization/[>=%s]@%s" % (self.version, userChannel))

        self.requires("ubitrack_tools_trackman/[>=1.0]@ubitrack/stable")

            

        #self.requires("ubitrack_uecamera/[>=%s]@%s" % (self.version, userChannel))


    def imports(self):
        self.copy(src="bin", pattern="*.dll", dst="./bin") # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib", pattern="*.dll", dst="./bin") # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib", pattern="*.dylib*", dst="./lib") # Copies all dylib files from packages lib folder to my "lib" folder
        self.copy(src="lib", pattern="*.so*", dst="./lib") # Copies all so files from packages lib folder to my "lib" folder
        self.copy(src="bin", pattern="ut*", dst="./bin") # Copies all applications
        self.copy(src="bin", pattern="log4cpp.conf", dst="./") # copy a logging config template
        self.copy(src="share/Ubitrack", pattern="*.*", dst="./share/Ubitrack") # copy all shared ubitrack files 
