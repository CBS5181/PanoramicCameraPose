#include "pch.h"
#include "WindowsPlatformUtils.h"
#include <Windows.h>
#include <shobjidl.h> // folder 
#include <commdlg.h> // WIN32 API
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h> // GLFWwindow -> HWND
#include "Application.h"


std::string FileDialogs::OpenFile(const char* filter)
{
	OPENFILENAMEA ofn; // common dialog box structure (ANSI version)
	CHAR szFile[260] = { 0 };
	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(OPENFILENAME));
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = glfwGetWin32Window(Application::Get().GetWindow());
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = filter;
	ofn.nFilterIndex = 1;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
	if (GetOpenFileNameA(&ofn) == TRUE)
	{
		return ofn.lpstrFile;
	}
	return std::string();
}

std::string FileDialogs::SaveFile(const char* filter)
{
	OPENFILENAMEA ofn; // common dialog box structure (ANSI version)
	CHAR szFile[260] = { 0 };
	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(OPENFILENAME));
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = glfwGetWin32Window(Application::Get().GetWindow());
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = filter;
	ofn.nFilterIndex = 1;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
	if (GetSaveFileNameA(&ofn) == TRUE)
	{
		return ofn.lpstrFile;
	}
	return std::string();
}

std::wstring FileDialogs::OpenFolder()
{
	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED |
		COINIT_DISABLE_OLE1DDE);

	
	if (SUCCEEDED(hr))
	{
		std::wstringstream ss;
		IFileOpenDialog* pFileOpen;

		// Create the FileOpenDialog object.
		hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL,
			IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));

		// Set options for a filesystem folder picker dialog.
		FILEOPENDIALOGOPTIONS opt{};
		pFileOpen->GetOptions(&opt);
		pFileOpen->SetOptions(opt | FOS_PICKFOLDERS | FOS_PATHMUSTEXIST | FOS_FORCEFILESYSTEM | FOS_NOCHANGEDIR);

		// Set Default Folder
		std::filesystem::path default_folder = std::filesystem::current_path() / L"assets\\test_data";
		LPCWSTR customPath = default_folder.c_str();
		IShellItem* location = nullptr;
		hr = SHCreateItemFromParsingName(customPath, nullptr, IID_PPV_ARGS(&location));
			
		if (SUCCEEDED(hr))
		{
			pFileOpen->SetFolder(location);
			// Show the Open dialog box.
			hr = pFileOpen->Show(NULL);
			// Get the folder from the dialog box.
			if (SUCCEEDED(hr))
			{
				IShellItem* pItem;
				hr = pFileOpen->GetResult(&pItem);
				if (SUCCEEDED(hr))
				{
					PWSTR pszFilePath;
					hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);

					ss << pszFilePath;
					CoTaskMemFree(pszFilePath);
					pItem->Release();
				}
			}
		}
		pFileOpen->Release();
		location->Release();
		CoUninitialize();
		return ss.str();
	}
	return std::wstring();
}