#include "PathSearch.h"

namespace fullsail_ai {
	namespace algorithms {
		PathSearch::PathSearch()
		{
			startingPlanner = nullptr;
			hWeight = 1.f;
		}

		PathSearch::~PathSearch()
		{
			shutdown();
		}
		void PathSearch::initialize(TileMap* _tileMap)
		{
			tileMap = _tileMap;
			int rowCount = tileMap->getRowCount();
			int colCount = tileMap->getColumnCount();

			// Create Search Nodes for each tile in the map
			for (int row = 0; row < rowCount; row++) {
				for (int column = 0; column < colCount; column++) {
					Tile* currentTile = tileMap->getTile(row, column);
					if (currentTile->getWeight() != 0) {
						SearchNode* newNode = new SearchNode;
						newNode->tile = currentTile;
						nodes[currentTile] = newNode;  
					}
				}
			}

			// Adding Neighbors
			const int offsetsEven[6][2] = { {-1, -1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {-1, 0} };
			const int offsetsOdd[6][2] = { {0, -1}, {0, 1}, {-1, 1}, {1, 0}, {1, 1}, {-1, 0} };

			for (int nodeRow = 0; nodeRow < rowCount; nodeRow++) {
				for (int nodeColumn = 0; nodeColumn < colCount; nodeColumn++) {
					Tile* currentTile = tileMap->getTile(nodeRow, nodeColumn);
					if (currentTile->getWeight() == 0) {
						continue;
					}

					SearchNode* tempSearch = nodes[currentTile];
					const int(*offsets)[2] = (nodeRow % 2 == 0) ? offsetsEven : offsetsOdd;

					for (int i = 0; i < 6; ++i) {
						int neighborRow = nodeRow + offsets[i][0];
						int neighborColumn = nodeColumn + offsets[i][1];

						if (neighborRow >= 0 && neighborRow < rowCount &&
							neighborColumn >= 0 && neighborColumn < colCount) {

							Tile* neighborTile = tileMap->getTile(neighborRow, neighborColumn);
							if (neighborTile->getWeight() != 0) {
								SearchNode* neighborNode = nodes[neighborTile];
								tempSearch->neighbors.push_back(neighborNode);
							}
						}
					}
				}
			}
		}

		float PathSearch::calculateHeuristicCost(SearchNode* current, SearchNode* goal) const
		{
			float dx = current->tile->getRow() - goal->tile->getRow();
			float dy = current->tile->getColumn() - goal->tile->getColumn();

			return sqrt(dx * dx + dy * dy);
		}

		void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
		{
			SearchNode* startingNode = nodes.find(tileMap->getTile(startRow, startColumn))->second;
			goalNode = nodes.find(tileMap->getTile(goalRow, goalColumn))->second;

			PlannerNode* startingPlanner = new PlannerNode;

			startingPlanner->searchNode = startingNode;
			startingPlanner->parent = nullptr;

			startingPlanner->givenCost = 0;
			startingPlanner->heuristicCost = calculateHeuristicCost(startingNode, goalNode);
			startingPlanner->finalCost = startingPlanner->givenCost + startingPlanner->heuristicCost * hWeight;

			openQ.push(startingPlanner);
			visited.insert(std::make_pair(startingNode, startingPlanner));
		}

		void PathSearch::update(long timeslice)
		{
			PlannerNode* currentPlanner = openQ.front();
			if (timeslice == 0) {
				openQ.pop();

				if (currentPlanner->searchNode == goalNode) {
					while (currentPlanner->parent != nullptr) {
						SolutionVector.push_back(currentPlanner->searchNode->tile);
						currentPlanner = currentPlanner->parent;
					}
					SolutionVector.push_back(currentPlanner->searchNode->tile);
				}
				else {
					for (SearchNode* neighbor : currentPlanner->searchNode->neighbors)
					{
						auto VisitedIter = visited.find(neighbor);
						float NeighborCost = currentPlanner->givenCost + neighbor->tile->getWeight();
						//neighborPlanner->searchNode = neighbor;

						if (visited.find(neighbor) != visited.end()) {

							PlannerNode* existingNode = VisitedIter->second;

							if (NeighborCost < existingNode->givenCost)
							{
								openQ.remove(existingNode);
								existingNode->givenCost = NeighborCost;
								existingNode->finalCost = existingNode->givenCost + existingNode->heuristicCost * hWeight;
								existingNode->parent = currentPlanner;
								openQ.push(existingNode);
							}
							
						}
						else
						{
							PlannerNode* neighborPlanner = new PlannerNode;
							neighborPlanner->searchNode = neighbor;
							neighborPlanner->givenCost = NeighborCost;
							neighborPlanner->heuristicCost = calculateHeuristicCost(neighbor, goalNode);
							neighborPlanner->finalCost = neighborPlanner->givenCost + neighborPlanner->heuristicCost * hWeight;
							neighborPlanner->parent = currentPlanner;

							visited.insert(std::make_pair(neighbor, neighborPlanner));
							neighborPlanner->searchNode->tile->setFill(0XFF855095);
							openQ.push(neighborPlanner);
							//delete neighborPlanner;
						}
					}
				}

				if (SolutionVector.size() > 0) {
					return;
				}
			}
			else {
				while (openQ.empty())
				{
					PlannerNode* currentPlanner = openQ.front();
					openQ.pop();

					if (currentPlanner->searchNode == goalNode) {
						while (currentPlanner->parent != nullptr) {
							SolutionVector.push_back(currentPlanner->searchNode->tile);
							currentPlanner = currentPlanner->parent;
						}
						SolutionVector.push_back(currentPlanner->searchNode->tile);
						return;
					}
					else {
						PlannerNode* neighborPlanner = new PlannerNode();
						for (SearchNode* neighbor : currentPlanner->searchNode->neighbors)
						{
							float NeighborCost = currentPlanner->givenCost + neighbor->tile->getWeight();

							if (visited.find(neighbor) != visited.end()) {
								if (NeighborCost < visited.find(neighbor)->second->givenCost)
								{
									neighborPlanner->searchNode = neighbor;
									openQ.remove(neighborPlanner);
									neighborPlanner->givenCost = NeighborCost;
									neighborPlanner->finalCost = neighborPlanner->givenCost + neighborPlanner->heuristicCost * hWeight;
									neighborPlanner->parent = currentPlanner;
									openQ.push(neighborPlanner);
								}
							}
							else
							{
								neighborPlanner->searchNode = neighbor;
								neighborPlanner->givenCost = NeighborCost;
								neighborPlanner->heuristicCost = calculateHeuristicCost(neighbor, goalNode);
								neighborPlanner->finalCost = neighborPlanner->givenCost + neighborPlanner->heuristicCost * hWeight;
								neighborPlanner->parent = currentPlanner;
								openQ.push(neighborPlanner);

								visited.insert(std::make_pair(neighbor, neighborPlanner));
							}
						}
					}
				}
			}
		}

		void PathSearch::exit()
		{
			while (!openQ.empty()) {
				delete openQ.front();
				openQ.pop();
			}

			
			visited.clear();
			SolutionVector.clear();
		}


		void PathSearch::shutdown()
		{
			//HAndling memory
			for (auto iter = nodes.begin(); iter != nodes.end();)
			{
				delete iter->second;
				iter = nodes.erase(iter);
			}
			nodes.clear();
		}

		bool PathSearch::isDone() const
		{
			if (SolutionVector.size() > 0) {
				return true;
			}

			return false;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			return SolutionVector;
		}
	}
}
// namespace fullsail_ai::algorithms