/*
Autor: Mateusz Pielach
Projekt wykonywany w ramach OSAD3D
Temat: IIE5 
wartoœæ intensywnoœci koloru danego punktu w podanym otoczeniu (parametr wejœciowy).
+ dodatkowo do ka¿dego tematu: zapis cech do warstw danych i podzia³ chmury na segmenty w
zale¿noœci od cech;

Data: 25.03.2019
Wersja: Beta_v1

*/


#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/KNNSearchKernel.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <ogx/Data/Primitives/PrimitiveHelpers.h>

using namespace ogx;
using namespace ogx::Data;

struct ColorIntensity : public ogx::Plugin::EasyMethod
{
	//fields
	Nodes::ITransTreeNode* m_node;

	//parameters
	Data::ResourceID m_node_id;

	int red_min, red_max;		// red color range
	int green_min, green_max;	// green color range
	int blue_min, blue_max;		// blue color range
	int m_neighbours_count;			// number of neighbours
	int intensity_min, intensity_max;

	//constructor
	ColorIntensity() : EasyMethod(L"Mateusz Pielach", L"Intensity detection - IIE5")
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

		bank.Add(L"intensity min", intensity_min = 0, L"Intensity minimum value (0-255)").Min(0).Max(255);
		bank.Add(L"intensity max", intensity_max = 255, L"Intensity maximum value (0-255)").Min(0).Max(255);

		bank.Add(L"node_id", m_node_id = Data::ResourceID::invalid).AsNode();	//cloud choice
		bank.Add(L"number of neighbours", m_neighbours_count = 10).Min(3);

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

			//get xyz values, color
			std::vector<Data::Clouds::Point3D> xyz_values;
			points_all.GetXYZ(xyz_values);
			std::vector<Data::Clouds::Color> color_original;
			points_all.GetColors(color_original);
			auto point = context.Feedback().GetFocusPoint();

			auto xyz_r = Data::Clouds::RangeLocalXYZConst(points_all);
			auto color_r = Data::Clouds::RangeColorConst(points_all);
			auto normal_r = Data::Clouds::RangeLocalNormalConst(points_all);
			auto state_r = Data::Clouds::RangeState(points_all);
			auto xyz = xyz_r.begin();
			auto state = state_r.begin();
			auto normal = normal_r.begin();
			auto color = color_r.begin();

			// initialize buffer for the output xyz coordinates
			std::vector<StoredPoint3D> buff_out;
			buff_out.reserve(points_all.size());


			////Layers processing

			//auto layer_list = cloud.FindLayers(L"layer_name");
			//if (layer_list.empty()) ReportError(L"No layers found");

			std::vector<StoredReal> layer_values; // store values

			for (auto & c : color_original)
			{
				layer_values.push_back(double(1.0 / 3.0 * (c[0] + c[1] + c[2])));	// RGB to grayscale conversion
			}

			auto v_sLayerName = L"intensity layer";
			Data::Layers::ILayer *layer;
			auto layers = cloud.FindLayers(v_sLayerName);
			// check if already exist layer
			if (!layers.empty())
			{
				layer = layers[0];
			}
			else
			{
				layer = cloud.CreateLayer(v_sLayerName, 0); // 0 - default value
			}

			points_all.SetLayerVals(layer_values, *layer); // saving layer to cloud


			////Neighbours
			int points_found = 0;




			//auto sphere = Math::CalcBestSphere3D(xyz_values.begin(), xyz_values.end());


			////////////////////////////////Neighbour search

			// initialize search over neighboring points (spherical kernel can be used instead Data::Clouds::SphericalSearchKernel)
			Data::Clouds::KNNSearchKernel search_knn(Math::Point3D::Zero(), m_neighbours_count);

			int iteration = 0;

			// search for N points around the current point
			Data::Clouds::PointsRange neighbours;	//points from neighbourhood
			std::vector<Data::Clouds::Point3D> neighbours_xyz_values;
			std::vector<Data::Clouds::Color> neighbours_color;		//color values 

			auto neighbours_state_range = Data::Clouds::RangeState(points_all);
			auto neighbour_state = neighbours_state_range.begin();

			std::vector<StoredReal> intensity_values;		//store gray values
			auto intensity = intensity_values.begin();

			for (auto& xyz : Data::Clouds::RangeLocalXYZConst(points_all)) 
			{
				// clear parameters from last iteration
				state->reset();
				neighbours.clear();
				neighbours_xyz_values.clear();
				neighbours_color.clear();
				intensity_values.clear();

				iteration++; // debug
				// update the center of the search
				search_knn.GetPoint() = xyz.cast<Math::Point3D::Scalar>();

				// search for N points around the current point
				cloud.GetAccess().FindPoints(search_knn, neighbours);
				//now I can access points from neighbourhood


				////get neighbours' xyz and colors
				neighbours.GetXYZ(neighbours_xyz_values);
				neighbours.GetColors(neighbours_color);
				neighbours.GetLayerVals(intensity_values, *layer);	//get layer values to vector intensity_values

				//debug
				//OGX_LINE.Format(ogx::Debug, L"%d Ejej", points_found);

				//OGX_LINE.Format(ogx::Info, L"%d iteracja", iteration);

				intensity_values.reserve(neighbours.size());

				//Trzeba zrobiæ tak ¿e liczy w skali szarosci srednia z otoczenia i wstawia t¹ wartoœæ w ten nowy punkt!

				// For all points included in the neighbourhood
				for (auto & value : Data::Clouds::RangeLayer(neighbours, *layer))
				{
					if((value > intensity_min) && (value < intensity_max))
					state->set(Data::Clouds::PS_SELECTED); //select point (from original cloud)
				}

				//////loop which looks for points which are within color range 
				//for (auto & c : neighbours_color)
				//{
				//	//state->reset(); //reset states (bring back original state for each point)
				//	if ((c[0] > red_min) && (c[1] > green_min) && (c[2] > blue_min) && (c[0] < red_max) && (c[1] < green_max) && (c[2] < blue_max))
				//	{
				//		state->set(Data::Clouds::PS_SELECTED); //select point (from original cloud)
				//		points_found++;
				//	}
				//	neighbour_state++; //move to next point (neighbourhood)
				//}


				//debug
				//OGX_LINE.Format(ogx::Debug, L"%d points were found", points_found);

				//// fit a plane to the neighboring points and calculate projection of the current point on the plane
				//Math::Point3D proj_xyz = Math::ProjectPointOntoPlane(Math::CalcBestPlane3D(neighbor_xyz.begin(), neighbor_xyz.end()), xyz.cast<Math::Point3D::Scalar>());
				//buff_out.push_back(proj_xyz.cast<StoredPoint3D::Scalar>());
				state++;
			}

			OGX_LINE.Format(ogx::Debug, L"%d points were found", points_found);

			// replace the cloud xyz coordinates with the smoothed ones
			//points_all.SetXYZ(buff_out);


			//debug
			OGX_LINE.Format(ogx::Debug, L"%d iteracji", iteration);
		});
	}

};

OGX_EXPORT_METHOD(ColorIntensity)