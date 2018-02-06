# coding:utf-8
"""
卒業研究用
道路ネットワークの自動生成と経路探索プログラム
"""

import networkx as nx
import matplotlib.pyplot as plt
import pandas as pd
import sys
import copy
import pickle
import random
import math


class RoadNetwork(nx.Graph):
    """道路ネットワークシミュレータ

    Examples
    --------
    >>> from road_network import RoadNetwork
    """

    def __init__(self):
        """RoadNetworkクラスの初期化

        Examples
        --------
        >>> net = RoadNetwork()
        """
        super().__init__()  # スーパークラスの初期化
        self.pos = None  # ノード座標の初期化
        self.start_node = 0  # 出発ノードの初期化
        self.end_node = 0 # 到着ノードの初期化

    def __nodes_distance(self, node1, node2):
        """ノード間の距離を返す

        Parameters
        ----------
        node1: list
            ノード１
        node2: list
            ノード２

        Returns
        -------
        nodes_distance: float
            ノード間の距離。
        """
        return math.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2)

    def __path_result(self, transit_nodes, result, weight):
        """探索結果のの描画

        Parameters
        ----------
        transit_nodes: list
            探索する経路のリスト
        result: list
            探索結果の経路のリスト
        weight: float
            重さ（距離）
        """
        node_count = len(self.nodes())
        result_edges = []

        for i in range(len(result)):
            if result[i] != result[-1]:
                result_edges.append((result[i], result[i + 1]))
            else:
                break

        # データの描画
        plt.title("transit_nodes: {}\nresult: {}\nweight: {}".format(
            transit_nodes, result, weight))

        # ノード番号の描画
        nx.draw_networkx_labels(self, self.pos, label=(
            range(node_count)), font_size=8, font_color='white')

        # ノードの描画
        nx.draw_networkx_nodes(
            self, self.pos, node_color='black', node_size=100)

        # 探索結果の経路で通るノードの描画
        nx.draw_networkx_nodes(
            self, self.pos, nodelist=result, node_color='red', node_size=150)

        # エッジの描画
        nx.draw_networkx_edges(self, self.pos, alpha=0.2)

        # 探索結果の経路で通るエッジの描画
        nx.draw_networkx_edges(
            self, self.pos, edgelist=result_edges, width=2, alpha=1, edge_color='red')

        # 表示
        plt.axis('off')
        plt.show()

    def __astar_algorithm(self, start_node, end_node):
        """A*アルゴリズムによる経路探索と重みの計算

        Parameters
        ----------
        start_node: int
            出発点のノード
        end_node: int
            到着点のノード

        Returns
        -------
        astar_result: tuple
            探索結果の経路と重みのtuple
        """
        result_route = nx.astar_path(
            self, start_node, end_node)  # A*アルゴリズムによる探索結果の経路
        result_weight = nx.astar_path_length(
            self, start_node, end_node)  # A*アルゴリズムによる探索結果の経路の重み
        return result_route, result_weight

    def set_start_node(self, start_node):
        """出発ノードのセット

        Parameters
        ----------
        start_node: int
            出発ノード

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.set_start_node(0)
        """
        self.start_node = start_node

    def set_end_node(self, end_node):
        """到着ノードのセット

        Parameters
        ----------
        end_node: int
            到着ノード

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.set_end_node(99)
        """
        self.end_node = end_node

    def generate_network(self, node_count):
        """道路ネットワークの生成

        Parameters
        ----------
        node_count: int
            総ノード数

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.generate_network(100)
        """
        if node_count > 1000:
            raise ValueError('Please enter a value less than 1000')

        super().__init__()

        # ノードの生成
        self.add_node(0, position=(random.random(), random.random()))
        count = 1
        while count < node_count:
            position = [random.random(), random.random()]
            nodes_size = len(self.nodes())

            for node in self.nodes(data=True):
                if self.__nodes_distance(position, node[1]['position']) <= math.log(node_count)/node_count:
                    break

                if node == self.nodes(data=True)[-1]:
                    self.add_node(count, position=position)

            if nodes_size < len(self.nodes()):
                count += 1

        # エッジの生成
        edges = []
        while len(edges) < node_count * 2:
            source = random.choice(self.nodes(data=True))
            target = random.choice(self.nodes(data=True))

            if self.__nodes_distance(source[1]['position'], target[1]['position']) <= math.log(node_count)/math.sqrt(node_count):
                edge = (source[0], target[0], self.__nodes_distance(
                    source[1]['position'], target[1]['position']))
            else:
                continue

            if (edge not in edges) and (edge[0] != edge[1]):
                edges.append(edge)

        self.add_weighted_edges_from(edges)

        # ノードの位置をインスタンス変数に保持
        self.pos = nx.get_node_attributes(self, 'position')

        self.start_node = self.nodes()[0]
        self.end_node = self.nodes()[-1]

    def load_network_pickle(self, file_name):
        """Pickleファイルから道路ネットワークの読み込み

        Parameters
        ----------
        file_name: str
            保存されたネットワークのファイル名（Pickleファイル）

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.load_network_pickle('road_network.pkl')
        """
        try:
            with open(file_name, 'rb') as f:
                copy_net = pickle.load(f)
        except FileNotFoundError:
            print(file_name + ' is not found...')
        else:
            # 読み込んだ道路ネットワークのインスタンス変数をコピー
            self.pos = copy.deepcopy(copy_net.pos)
            self.node = copy.deepcopy(copy_net.node)
            self.edge = copy.deepcopy(copy_net.edge)
            self.graph = {}
            self.adj = copy.deepcopy(copy_net.adj)
            self.start_node = copy_net.start_node
            self.end_node = copy_net.end_node

            # 読み込んだ道路ネットワークオブジェクトを削除
            del copy_net

    def load_network_csv(self, nodes_csv, edges_csv):
        """CSVファイルから道路ネットワークの読み込み

        Parameters
        ----------
        nodes_csv: str
            ノードのCSVファイル
        edges_csv: str
            エッジのCSVファイル

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.load_network_csv('nodes.csv', 'edges.csv')
        """
        # インスタンス変数のリフレッシュ
        super().__init__()
        self.pos = None
        self.start_node = 0

        # ノードデータの読み込み
        nodes_df = pd.read_csv(nodes_csv)
        for _, row in nodes_df.iterrows():
            self.add_node(int(row.node), position=(
                row.position_x, row.position_y))

        # エッジデータの読み込み
        edges_df = pd.read_csv(edges_csv)
        edges = []
        for _, row in edges_df.iterrows():
            edges.append((int(row.source), int(row.target), row.weight))
        self.add_weighted_edges_from(edges)

        # ノードの位置をインスタンス変数に保持
        self.pos = nx.get_node_attributes(self, 'position')

        self.start_node = self.nodes()[0]
        self.end_node = self.nodes()[-1]

    def save_network_pickle(self, file_name):
        """道路ネットワークをPickleファイルとして保存

        Parameters
        ----------
        file_name: str
            ネットワークのファイル名（Pickleファイル）

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.generate_network(100)
        >>> net.save_network('road_network.pkl')
        """
        with open(file_name, 'wb') as f:
            pickle.dump(self, f)

    def save_network_csv(self, node_file_name, edge_file_name):
        """道路ネットワークをCSVファイルとして保存

        Parameters
        ----------
        nodes_csv: str
            ノードのCSVファイル
        edges_csv: str
            エッジのCSVファイル

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.generate_network(100)
        >>> net.save_network_csv('nodes.csv', 'edges.csv')
        """
        nodes_list = []
        for node in self.nodes(data=True):
            nodes_list.append(
                [int(node[0]), node[1]['position'][0], node[1]['position'][1]])
        df = pd.DataFrame(nodes_list, columns=[
                          'node', 'position_x', 'position_y'])
        df.to_csv(node_file_name, index=False)

        edges_list = []
        for edge in self.edges(data=True):
            edges_list.append([int(edge[0]), int(edge[1]), edge[2]['weight']])
        df = pd.DataFrame(edges_list, columns=['source', 'target', 'weight'])
        df.to_csv(edge_file_name, index=False)

    def show_network(self):
        """道路ネットワークの描画

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.generate_network(100)
        >>> net.show_network()
        """
        plt.axis('off')

        # ノード番号の描画
        nx.draw_networkx_labels(self, self.pos, label=(
            range(len(self.nodes()))), font_size=8, font_color='white')

        # ノードの描画
        nx.draw_networkx_nodes(
            self, self.pos, node_color='black', node_size=100)

        # エッジの描画
        nx.draw_networkx_edges(self, self.pos, alpha=0.2)

        # 表示
        plt.show()

    def route_search(self, transit_nodes):
        """道路ネットワークの経路探索

        Parameters
        ----------
        transit_nodes: list
            通過ノードのlist

        Returns
        -------
        result_dict: dict
            探索する経路、探索した結果の経路、重み（探索した結果の経路の総距離）のdict

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.generate_network(100)
        >>> net.route_search([23, 56, 65])
        {'transit_nodes': [23, 56, 65],
         'result_route': [0, 96, 22, 49, 74, 23, 74, 90, 56, 86, 8, 60, 65],
         'result_weight': 3.0993069396450568}
        """
        copy_transit_nodes = copy.deepcopy(transit_nodes)  # 探索する経路のコピー
        result_route = [self.start_node]  # 探索結果の経路を格納するlist
        result_weight = 0  # 探索経過の経路の重みを格納するlist

        # 探索
        start = self.start_node
        while len(transit_nodes) >= 1:
            weight = sys.maxsize
            for node in transit_nodes:
                get_route, get_weight = self.__astar_algorithm(start, node)
                if get_weight < weight:
                    result_weight += get_weight
                    weight = get_weight
                    start = node
                    result_route += get_route[1:]
                    transit_nodes.remove(node)
        get_route, get_weight = self.__astar_algorithm(start, self.end_node)
        result_weight += get_weight
        result_route += get_route[1:]
        return {'transit_nodes': copy_transit_nodes,
                'result_route': result_route,
                'result_weight': result_weight}

    def draw_search_route(self, transit_nodes):
        """道路ネットワークの経路探索後、描画

        Parameters
        ----------
        transit_nodes: list
            通過ノードのlist

        Examples
        --------
        >>> net = RoadNetwork()
        >>> net.generate_network(100)
        >>> net.draw_search_route([23, 56, 65])
        """
        result = self.route_search(transit_nodes)
        self.__path_result(result['transit_nodes'],
                           result['result_route'],
                           result['result_weight'])
