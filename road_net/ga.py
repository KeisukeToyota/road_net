import networkx as nx
from random import choice, random, randint
import copy
from road_network import RoadNetwork

individual_lenght = 300 # 個体数
generation = 100 # 世代数
mutate_rate = 0.2 # 突然変異の確率
elite_rate = 0.1 # エリート選択の割合

routes = [0, 1, 2, 3, 4, 5]

def weight(net, start, end):
    """重み計算
    :param net: RoadNetworkインスタンス
    :param start: 始発ノード
    :param end: 終点ノード
    :return: 重み
    """
    result_weight = nx.astar_path_length(net, start, end)
    return result_weight


def get_population(net):
    """初期個体群生成
    :param net: RoadNetworkインスタンス
    :return: 初期個体群
    """
    population = []

    def f(net):
        count = len(net.nodes())
        edges = net.edges(data=True)

        individual = []
        for i in range(count):
            genetic = []
            for j in edges:
                if i == j[0]:
                    genetic.append(j[1])
                elif i == j[1]:
                    genetic.append(j[0])
            individual.append(choice(genetic))
        return individual

    while len(population) < individual_lenght:
        individual = f(net)
        r = []
        route = routes[0]
        while True:
            if individual[route] in r:
                break
            r.append(individual[route])
            route = individual[route]
        if list_check(routes[1:], individual):
            population.append([999, individual])

    return population


def mutate(parent, edges):
    """突然変異
    :param parent: 親個体
    :param edges: エッジ
    :return: 子個体
    """
    r = randint(0, len(parent)-1)
    child = copy.deepcopy(parent)
    arr = []
    for edge in edges:
        if child[r] in edge:
            if edge[0] == r:
                arr.append(edge[1])
            elif edge[1] == r:
                arr.append(edge[0])
    child[r] = choice(arr)
    return child


def crossover(parent1, parent2, net):
    """２点交叉
    :param parent1: 親個体１
    :param parent2: 親個体２
    :param net: RoadNetworkインスタンス
    :return: 子個体
    """
    r1 = randint(0, len(parent1)-1)
    r2 = randint(r1, len(parent2)-1)
    child = copy.deepcopy(parent1)
    child[r1:r2] = parent2[r1:r2]
    return child


def list_check(list1, list2):
    """リストチェック関数
    :param list1: リスト１
    :param list2: リスト２
    :return: 真偽値
    """
    return set(list1).issubset(list2)


def elite(pop, routes=routes):
    """エリート選択
    :param pop: 個体群
    :param routes: 経由ルート
    :return: エリート個体群
    """
    arr = []
    for p in pop:
        r = []
        route = routes[0]
        while True:
            if r.count(route) >= 2:
                break
            r.append(p[1][route])
            route = p[1][route]
        if list_check(routes[1:], r):
            arr.append(p)
    return arr


def fit(net, child, edges):
    """評価
    :param net: RoadNetworkインスタンス
    :param child: 子個体
    :param edges: エッジ
    :return: 重み
    """
    sum_w = 0.0
    r = []
    route = routes[0]
    while True:
        if child[route] in r:
            break
        r.append(route)
        route = child[route]
    if list_check(routes[1:], r):
        for i in range(len(r)):
            if i != len(r) - 1:
                sum_w += weight(net, r[i], r[i + 1])
    if sum_w <= 0:
        sum_w = 999

    return sum_w

def main(net):
    """メイン関数
    :param net: RoadNetworkインスタンス
    :return: 探索結果
    """
    weights = []
    edges = net.edges(data=True)
    print('Generation : 0')
    pop = get_population(net)
    pop.sort()
    weights.append(pop[0][0])

    for g in range(generation):
        r = elite(pop, routes)
        if len(r) <= 0:
            r = copy.deepcopy(pop)
            r.sort()
        pop = r[:int(len(pop)*elite_rate)]

        while len(pop) < individual_lenght:
            if random() < mutate_rate:
                m = randint(0, len(r)-1)
                child = mutate(r[m][1], edges)
            else:
                c1 = randint(0, len(r)-1)
                c2 = randint(0, len(r)-1)
                child = crossover(r[c1][1], r[c2][1], net)
            pop.append([fit(net, child, edges), child])
        pop.sort()
        if g % 10 == 0:
            print('Generation : ' + str(g))
            print('Min : {}'.format(pop[-1][0]))
            print('Max : {}'.format(pop[0][0]))
            print('--------------------------')
        weights.append(pop[0][0])
    result = pop
    result.sort()

    r = []
    route = routes[0]
    r.append(route)
    count = 0
    while True:
        r.append(result[0][1][route])
        route = result[0][1][route]
        if list_check(routes[1:], r):
            print(result[0][0])
            return r
        if count > len(result[0][1]):
            break
        count += 1

if __name__ == '__main__':
    net = RoadNetwork()
    net.load_network_pickle('ten_nodes.pkl')
    r = main(net)
    print(r) # 探索結果の経路出力
