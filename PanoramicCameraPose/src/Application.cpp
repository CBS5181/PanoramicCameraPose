#include "pch.h"
#include "Application.h"
#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "PanoLayer.h"

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

bool firstMouse = true;
bool canMouseRotate = false;
static bool isFirstTime = true;

float lastX = 512.0;
float lastY = 256.0;

static bool isDragDrop = false;
std::filesystem::path g_FilePath;

Application* Application::s_Instance = nullptr;

Application::Application(const ApplicationSpecification& applicationSpecification)
	: m_Specification(applicationSpecification)
{
	assert(!s_Instance); // Application already exists!
	s_Instance = this;
	Init();
}

Application::~Application()
{
	Shutdown();
}

void Application::Run()
{
	m_Running = true;
	ImGuiIO& io = ImGui::GetIO();

	while (!glfwWindowShouldClose(m_WindowHandle) && m_Running)
	{
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		processInput(m_WindowHandle);
		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		{
			static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

			// We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
			// because it would be confusing to have two docking targets within each others.
			ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
			if (m_MenubarCallback)
				window_flags |= ImGuiWindowFlags_MenuBar;


			const ImGuiViewport* viewport = ImGui::GetMainViewport();
			ImGui::SetNextWindowPos(viewport->WorkPos);
			ImGui::SetNextWindowSize(viewport->WorkSize);
			ImGui::SetNextWindowViewport(viewport->ID);
			ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
			ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
			window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
			window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;


			// When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
			// and handle the pass-thru hole, so we ask Begin() to not render a background.
			if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
				window_flags |= ImGuiWindowFlags_NoBackground;

			// Important: note that we proceed even if Begin() returns false (aka window is collapsed).
			// This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
			// all active windows docked into it will lose their parent and become undocked.
			// We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
			// any change of dockspace/settings would lead to windows being stuck in limbo and never being visible.
			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
			ImGui::Begin("DockSpace Demo", nullptr, window_flags);
			ImGui::PopStyleVar(3);

			// Submit the DockSpace
			ImGuiIO& io = ImGui::GetIO();
			ImGuiID dockspace_id = ImGui::GetID("DockSpace Demo");
			ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

			if (isFirstTime)
			{
				ImGui::DockBuilderRemoveNode(dockspace_id);
				ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace |
					ImGuiDockNodeFlags_PassthruCentralNode);
				ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

				static constexpr float ratioRight = 1.0f / 3.0f;

				ImGuiID PanoLayerID = -1;
				ImGuiID ToolLayerID = -1;
				// Right split so the central node is the left(viewport)
				ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, ratioRight, &ToolLayerID, &PanoLayerID);
				ImGui::DockBuilderDockWindow("Viewport", PanoLayerID);
				ImGui::DockBuilderDockWindow("Tool", ToolLayerID);
				ImGui::DockBuilderFinish(dockspace_id);
				isFirstTime = false;
			}
			else
			{
				ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);
			}

			if (m_MenubarCallback)
			{
				if (ImGui::BeginMenuBar())
				{
					m_MenubarCallback();
					ImGui::EndMenuBar();
				}
			}

			for (auto& layer : m_LayerStack) {
				layer->OnUpdate();
				layer->OnUIRender();
			}
			ImGui::End();
		}

		// Bug fix: drag and drop source should be submit after calling ImGui::NewFrame().
		{
			if (isDragDrop && ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceExtern))
			{
				const wchar_t* itemPath = g_FilePath.c_str();
				ImGui::SetDragDropPayload("FILES", itemPath, (wcslen(itemPath) + 1) * sizeof(wchar_t));
				ImGui::EndDragDropSource();
				isDragDrop = false;
			}
		}

		ImGui::EndFrame();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		// Update and Render additional Platform Windows
		// (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
		//  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
		if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
		{
			GLFWwindow* backup_current_context = glfwGetCurrentContext();
			ImGui::UpdatePlatformWindows();
			ImGui::RenderPlatformWindowsDefault();
			glfwMakeContextCurrent(backup_current_context);
		}

		glfwSwapBuffers(m_WindowHandle);
	}
}

void Application::Close()
{
	m_Running = false;
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(m_WindowHandle);
	glfwTerminate();
}

void Application::Init()
{
	std::cout << "Application Init..." << std::endl;
	// Initialize GLFW
	if (!glfwInit()) exit(EXIT_FAILURE);
	const char* glsl_version = "#version 460 core";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
	//glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
	
	//maximize window?
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	if (mode)
	{
		m_Specification.Width = mode->width;
		m_Specification.Height = mode->height;
	}

	// Open the window
	m_WindowHandle = glfwCreateWindow(m_Specification.Width, m_Specification.Height, m_Specification.Name.c_str(), NULL, NULL);
	if (!m_WindowHandle)
	{
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(m_WindowHandle);

	glfwSetWindowUserPointer(m_WindowHandle, &m_Data);

	// SetVSync
	glfwSwapInterval(1);

	glfwSetWindowSizeCallback(m_WindowHandle, window_resize_callback);
	//glfwSetFramebufferSizeCallback(m_WindowHandle, framebuffer_size_callback);
	glfwSetMouseButtonCallback(m_WindowHandle, mouse_button_callback);
	glfwSetCursorPosCallback(m_WindowHandle, mouse_callback);
	glfwSetScrollCallback(m_WindowHandle, scroll_callback);
	glfwSetKeyCallback(m_WindowHandle, key_press_callback);
	glfwSetDropCallback(m_WindowHandle, drop_callback);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return ;
	}

	int flags; glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
	if (flags & GL_CONTEXT_FLAG_DEBUG_BIT)
	{
		//std::cout << "DEBUG MODE\n";
		glEnable(GL_DEBUG_OUTPUT);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
		//glDebugMessageCallback(glDebugOutput, nullptr);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	}

	// Get Framebuffer size
	int fbw, fbh;
	glfwGetFramebufferSize(m_WindowHandle, &fbw, &fbh);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
	//io.ConfigViewportsNoAutoMerge = true;
	//io.ConfigViewportsNoTaskBarIcon = true;

	float fontSize = 28.0f;
	io.Fonts->AddFontFromFileTTF("assets/fonts/opensans/OpenSans-Bold.ttf", fontSize);
	io.FontDefault = io.Fonts->AddFontFromFileTTF("assets/fonts/opensans/OpenSans-Regular.ttf", fontSize);
	io.FontAllowUserScaling = true;

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
	ImGuiStyle& style = ImGui::GetStyle();
	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	{
		style.WindowRounding = 0.0f;
		style.Colors[ImGuiCol_WindowBg].w = 1.0f;
	}

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(m_WindowHandle, true);
	ImGui_ImplOpenGL3_Init(glsl_version);
}

void Application::Shutdown()
{
	for (auto& layer : m_LayerStack)
		layer->OnDetach();

	m_LayerStack.clear();

}

void Application::window_resize_callback(GLFWwindow* window, int width, int height)
{
	WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
	data.Width = width;
	data.Height = height;
	glViewport(0, 0, width, height);
}


// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void Application::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void Application::processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	/*if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		PanoLayer::cam.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		PanoLayer::cam.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		PanoLayer::cam.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		PanoLayer::cam.ProcessKeyboard(RIGHT, deltaTime);*/
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------

void Application::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{

	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;
	if (canMouseRotate)
	{
		//PanoLayer::cam.ProcessMouseMovement(xoffset, yoffset);
		//PanoLayer::cam2.ProcessMouseMovement(xoffset, yoffset);
	}

}

void Application::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	//scene.MouseButtonCallback(window, button, action, mods);
	//Input::MouseButtonCallback(window, button, action, mods);
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		canMouseRotate = true;
	}
	else
	{
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		canMouseRotate = false;
	}

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		PanoLayer::isMouseButtonLeftClick = true;
	}
	else
	{
		PanoLayer::isMouseButtonLeftClick = false;
	}
}


void Application::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	//PanoLayer::cam.ProcessMouseScroll(yoffset);
	//PanoLayer::cam2.ProcessMouseScroll(yoffset);
}

void Application::key_press_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	//scene.KeyPressCallback(window, key, scancode, action, mods);
	//Input::KeyCallback(window, key, scancode, action, mods);
}

void Application::drop_callback(GLFWwindow* window, int count, const char** paths)
{
	// current support only one file drop
	g_FilePath = paths[0];
	isDragDrop = true;
}
	