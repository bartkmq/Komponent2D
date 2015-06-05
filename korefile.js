var solution = new Solution('Komponent2D');
var project = new Project('Komponent2D');
project.addFiles('Kha/Backends/Kore/khacpp/src/**.h', 'Kha/Backends/Kore/khacpp/src/**.cpp', 'Kha/Backends/Kore/khacpp/include/**.h', 'Kha/Backends/Kore/khacpp/project/libs/common/**.h', 'Kha/Backends/Kore/khacpp/project/libs/common/**.cpp', 'Kha/Backends/Kore/khacpp/project/libs/msvccompat/**.cpp', 'Kha/Backends/Kore/khacpp/project/libs/regexp/**.h', 'Kha/Backends/Kore/khacpp/project/libs/regexp/**.cpp', 'Kha/Backends/Kore/khacpp/project/libs/std/**.h', 'Kha/Backends/Kore/khacpp/project/libs/std/**.cpp', 'Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8/**.h', 'Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8/**.c', 'Kha/Backends/Kore/*.cpp', 'Kha/Backends/Kore/*.h', 'build/windows-build/Sources/**.h', 'build/windows-build/Sources/**.cpp', 'build/windows-build/Sources/**.metal');
project.addExcludes('Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8/dftables.c', 'Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8/pcredemo.c', 'Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8/pcregrep.c', 'Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8/pcretest.c', 'Kha/Backends/Kore/khacpp/src/ExampleMain.cpp', 'Kha/Backends/Kore/khacpp/src/hx/Scriptable.cpp', 'Kha/Backends/Kore/khacpp/src/hx/cppia/**', '**/src/__main__.cpp', 'Kha/Backends/Kore/khacpp/src/hx/NekoAPI.cpp');
project.addIncludeDirs('Kha/Backends/Kore/khacpp/include', 'build/windows-build/Sources/include', 'Kha/Backends/Kore/khacpp/project/thirdparty/pcre-7.8', 'Kha/Backends/Kore/khacpp/project/libs/nekoapi');
project.setDebugDir('build/windows');
project.addDefine('HX_WINDOWS');
project.addDefine('HXCPP_API_LEVEL=321');
project.addDefine('STATIC_LINK');
project.addDefine('PCRE_STATIC');
project.addDefine('HXCPP_SET_PROP');
project.addDefine('HXCPP_VISIT_ALLOCS');
project.addDefine('KORE');
project.addDefine('ROTATE90');
project.addDefine('_WINSOCK_DEPRECATED_NO_WARNINGS');
project.addLib('ws2_32');
project.addSubProject(Solution.createProject('Kha/Kore'));
solution.addProject(project);
if (fs.existsSync('Libraries')) {
	var libraries = fs.readdirSync('Libraries');
	for (var l in libraries) {
		var lib = libraries[l];
		if (fs.existsSync(path.join('Libraries', lib, 'korefile.js'))) {
			project.addSubProject(Solution.createProject('Libraries/' + lib));
		}
	}
}
return solution;
