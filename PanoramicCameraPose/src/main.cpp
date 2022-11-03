#include "pch.h"
#include "Application.h"

extern Application* CreateApplication(int argc, char** argv);

int main(int argc, char** argv)
{

	Application* app = CreateApplication(argc, argv);
	app->Run();
	delete app;
	return 0;
}