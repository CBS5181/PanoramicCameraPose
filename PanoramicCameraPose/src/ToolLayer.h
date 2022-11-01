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
	int match_ind = 0;
};