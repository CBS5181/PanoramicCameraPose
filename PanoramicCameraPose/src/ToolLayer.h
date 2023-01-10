#pragma once
#include "Layer.h"
#include "MatchPoints.h"
#include "FileManager.h"
#include <vector>

class ToolLayer : public Layer
{
public:
	ToolLayer();
	virtual void OnUIRender() override;
	static MatchPoints s_MatchPoints;
	static FileManager s_FileManager;

private:
	std::vector<glm::vec3> m_PanoPos_gt;
	
	//generate more matchings by: 1) subdividing current matching in circular or not way
	//2) generte more matchings on adjacent "ceiling" and "floor" planes
	void PopulateMatching(bool circular);
};

//add a line to g_texts_to_show
void AddTextToShow(const char* str);