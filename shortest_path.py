# Copyright: Pouyan B. Navard @ PCVLab @ OSU 
import itertools
import numpy
import cv2
from PIL import Image 
from scipy.sparse.dok import dok_matrix
from scipy.sparse.csgraph import dijkstra as scipy_dijkstra
import matplotlib.pyplot as plt


class Dijkstra():
    def __init__(self):
        pass 

    def mouse_callback(self):
        # set mouse callback for the starting & destination points to be defined
        self.src_dst_points = []
        cv2.namedWindow("BinaryMap")
        cv2.setMouseCallback("BinaryMap", self.set_start_dst)
        cv2.imshow("BinaryMap",self.thresh_img)
        cv2.waitKey(0)
        
    def set_image(self, im_path, set_point = True):
        self.org_img = cv2.imread(im_path)
        self.gray_img = cv2.cvtColor(self.org_img, cv2.COLOR_BGR2GRAY)
        _, self.thresh_img = cv2.threshold(self.gray_img, 0, 255, cv2.THRESH_BINARY)
        self.binary_map = self.thresh_img.astype(bool)
        if set_point:
            self.mouse_callback()

    def to_index(self, y, x):
        "Defines a translation from 2 coordinates to a single number" 
        return y * self.org_img.shape[1] + x

    def to_coordinates(self, index):
        "Defines a reversed translation from index to 2 coordinates"
        return index // self.org_img.shape[1], index % self.org_img.shape[1]

    def set_start_dst(self, event, x, y, flags, parameters):
        "Setting the starting and destination points for the dijkstra"
        if event==cv2.EVENT_LBUTTONDBLCLK:
            self.src_dst_points.append((int(x),int(y)))

        if self.src_dst_points.__len__()==2:
            cv2.destroyAllWindows()


    def adjacency_mat(self):
        "A sparse adjacency matrix representing walkable and walkable path"
        # getting the indices of the center of each pathway through distanceTransformImage so that the walls/boundaries are avoided
        dist_tran_img, cnt_path_ind= self.find_cnt_path()
        # Two pixels are adjacent in the graph if both are painted.
        adjacency = dok_matrix((self.thresh_img.shape[0] * self.thresh_img.shape[1],
                                self.thresh_img.shape[0] * self.thresh_img.shape[1]), dtype=numpy.float64)


        # Filling the adjacency matrix by boolean values 
        directions = list(itertools.product([0, 1, -1], [0, 1, -1]))

        for i in range(1, self.binary_map.shape[0] - 1):
            for j in range(1, self.binary_map.shape[1] - 1):
                # skip the pixel if it is false 
                if not self.binary_map[i, j]:
                    continue
                for y_diff, x_diff in directions:
                    # if there is a walkable pixel 
                    if self.binary_map[i + y_diff, j + x_diff]:
                        # set the special adjacency weight for this pixelas per the value returned by the distanceTransformimage
                        # scipy's dijsktra implementation minimizes the sum of the adjacency's matrix. So we invert the weights 
                        adjacency[self.to_index(i, j),
                                  self.to_index(i + y_diff, j + x_diff)] = 1/dist_tran_img[(i + y_diff, j + x_diff)]

        return adjacency

    def shortest_path(self, adjacency):
        " fidning the shortest path between two points using Dijkstra Fibonacci Heap"
        source = self.to_index(self.src_dst_points[0][1], self.src_dst_points[0][0])
        target = self.to_index(self.src_dst_points[1][1], self.src_dst_points[1][0])

        # Compute the shortest path between the source and all other points in the image
        _, predecessors = scipy_dijkstra(adjacency, directed=False, indices=[source],
                                            unweighted=False, return_predecessors=True)

        # Constructs the path between source and target
        pixel_index = target
        self.pixels_path = []
        while pixel_index != source:
            self.pixels_path.append(pixel_index)
            pixel_index = predecessors[0, pixel_index]

    def visualize_path(self, down_sample=2):
        "debugging and it visualizing the shortest path"
        # img = self.org_img.copy()
        img=self.thresh_img
        #  down sampling for nicer visualization 
        pixels_path = self.pixels_path[::down_sample]
        pts = []
        # shortest_path = []
        for pixel_index in pixels_path:
            i, j = self.to_coordinates(pixel_index)
            # shortest_path.append((i, j))
            img[i, j] = False
        # import pickle 
        # with open("shortes_path", "wb") as fp:
        #     pickle.dump(shortest_path, fp)

        cv2.namedWindow("ShortestPath")
        # add circle shapes representing starting and destination points 
        img = cv2.circle(img.astype(numpy.uint8), self.src_dst_points[0], 5, (0, 0, 255), -1)
        img = cv2.circle(img.astype(numpy.uint8), self.src_dst_points[1], 5, (0, 255, 0), -1)
        cv2.imshow("ShortestPath", img.astype(numpy.uint8))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def find_cnt_path(self):
        "find the center of hallway by replacing each pixel values by its distance to the nearest background pixel"
        dist_tran_img = cv2.distanceTransform(self.thresh_img, distanceType=1, maskSize=3)
        # finding the indices showing highlighting the center of each pathways 
        i,j= numpy.where(dist_tran_img!=0)
        center_path_ind= list(zip(*[i,j]))
        
        # cv2.namedWindow("DistanceTransform", cv2.WINDOW_NORMAL)
        # cv2.imshow("DistanceTransform",numpy.uint8(dist_tran_img))
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return dist_tran_img, center_path_ind



if __name__ == '__main__':
    dijkstra = Dijkstra()
    dijkstra.set_image("./data/map1/binarymap.png", True) 
    adjacency = dijkstra.adjacency_mat()
    dijkstra.shortest_path(adjacency)
    dijkstra.visualize_path()
    dijkstra.find_cnt_path()


