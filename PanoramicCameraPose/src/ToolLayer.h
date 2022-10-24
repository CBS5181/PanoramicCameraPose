#pragma once
#include "Layer.h"
#include "MatchPoints.h"
#include <vector>
class ToolLayer : public Layer
{
public:
	ToolLayer();
	virtual void OnUIRender() override;
	static MatchPoints s_MatchPoints;
private:
	std::vector<float> m_match_depth;
	std::vector<float> m_pano01_depth;
	int match_ind = 0;
};