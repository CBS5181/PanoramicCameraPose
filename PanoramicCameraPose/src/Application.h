#pragma once
#include <memory>
#include <functional>
#include "Layer.h"

struct GLFWwindow;

struct ApplicationSpecification
{
	std::string Name = "PanoramicCameraPose";
	uint32_t Width = 1600;
	uint32_t Height = 1100;
};


class Application
{
public:
	Application(const ApplicationSpecification& applicationSpecification = ApplicationSpecification());
	~Application();

	void Run();

	template<typename T>
	void PushLayer()
	{
		static_assert(std::is_base_of<Layer, T>::value, "Pushed type is not subclass of Layer!");
		m_LayerStack.emplace_back(std::make_shared<T>())->OnAttach();
	}

	void PushLayer(const std::shared_ptr<Layer>& layer) { m_LayerStack.emplace_back(layer); layer->OnAttach(); }
	static Application& Get() { return *s_Instance; }
	GLFWwindow* GetWindow() { return m_WindowHandle; }
	unsigned int GetWindowWidth() const { return m_Data.Width; }
	unsigned int GetWindowHeight() const { return m_Data.Height; }
	const ApplicationSpecification& GetSpecification() const { return m_Specification; }
	void Close();


	static void window_resize_callback(GLFWwindow* window, int width, int height);
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	static void key_press_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void drop_callback(GLFWwindow* window, int count, const char** paths);
	void processInput(GLFWwindow* window);

private:
	void Init();
	void Shutdown();

private:
	ApplicationSpecification m_Specification;
	GLFWwindow* m_WindowHandle = nullptr;
	bool m_Running = false;
	std::function<void()> m_MenubarCallback;
	std::vector<std::shared_ptr<Layer>> m_LayerStack;
private:
	static Application* s_Instance;

	struct WindowData
	{
		std::string Title;
		unsigned int Width, Height;
	};

	WindowData m_Data;
};