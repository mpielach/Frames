/*
Autor: Mateusz Pielach, 278231
E-mail: 278231@pw.edu.pl
Projekt wykonywany w ramach OSAD3D
Temat: IE4
Data: 28.03.2019
Wersja: 1.0
Filtracja koloru – filtracja okreœlonego przez u¿ytkownika (parametr wej) koloru i zaznaczenie punktów o podanym kolorze (mo¿na wybraæ zakres koloru)
*/


#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/KNNSearchKernel.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <ogx/Data/Primitives/PrimitiveHelpers.h>

using namespace ogx;
using namespace ogx::Data;


struct ColorFilter : public ogx::Plugin::EasyMethod
{
	//fields
	Nodes::ITransTreeNode* m_node;

	//parameters
	Data::ResourceID m_node_id;
	int red_min, red_max;		// red color range
	int green_min, green_max;	// green color range
	int blue_min, blue_max;		// blue color range
	bool delete_points;			// deletes points if true

	//constructor
	ColorFilter() : EasyMethod(L"Mateusz Pielach", L"Color filtration - IE4")
	{
	}

	//add input/output parameters
	virtual void DefineParameters(ParameterBank& bank)
	{
		bank.Add(L"red min", red_min = 0, L"Red channel minimum value (0-255)").Min(0).Max(255);
		bank.Add(L"red max", red_max = 255, L"Red channel maximum value (0-255)").Min(0).Max(255);
		bank.Add(L"green min", green_min = 0, L"Green channel minimum value (0-255)").Min(0).Max(255);
		bank.Add(L"green max", green_max = 255, L"Green channel maximum value (0-255").Min(0).Max(255);
		bank.Add(L"blue min", blue_min = 0, L"Blue channel minimum value (0-255)").Min(0).Max(255);
		bank.Add(L"blue max", blue_max = 255, L"Blue channel maximum value (0-255)").Min(0).Max(255);
		bank.Add(L"node id", m_node_id = Data::ResourceID::invalid).AsNode();	//cloud choice
		bank.Add(L"delete points", delete_points = false, L"If set, deletes filtered points");
	}

	bool Init(Execution::Context& context)
	{
		OGX_SCOPE(log);
		//get node from id
		m_node = context.m_project->TransTreeFindNode(m_node_id);
		if (!m_node) ReportError(L"You must define node_id");

		OGX_LINE.Msg(User, L"Initialization succeeded");
		return EasyMethod::Init(context);
	}

	virtual void Run(Context& context)
	{
		Data::Clouds::ForEachCloud(*m_node, [&](Clouds::ICloud & cloud, Nodes::ITransTreeNode & node)
		{
			//access points in the cloud
			Data::Clouds::PointsRange points_all;
			cloud.GetAccess().GetAllPoints(points_all);

			//get color values and states
			std::vector<Data::Clouds::Color> colors;
			points_all.GetColors(colors);
			auto state_range = Data::Clouds::RangeState(points_all);
			auto state = state_range.begin();
			auto point = context.Feedback().GetFocusPoint();

			int points_found = 0;

			//loop which looks for points which are within color range 
			for (auto & c : colors)
			{
				state->reset(); //bring back original state for each point
				if ((c[0] > red_min) && (c[1] > green_min) && (c[2] > blue_min) && (c[0] < red_max) && (c[1] < green_max) && (c[2] < blue_max))
				{
					state->set(Data::Clouds::PS_SELECTED); //select point
					if (delete_points)
					{
						state->set(Data::Clouds::PS_DELETED);	//delete point
					}
					points_found++;
				}
				state++;
			}
			OGX_LINE.Format(ogx::Info, L"%d points within selected range were found", points_found);
		});
	}
};

OGX_EXPORT_METHOD(ColorFilter)