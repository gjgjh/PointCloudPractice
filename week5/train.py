import os.path
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

import torch
import torch.nn.parallel
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as data
import torch.utils.data

from PointNet import PointNet

dataset = '/home/irsgis/dataset/modelnet40_normal_resampled'

num_points = 2500
batchSize = 32
nepoch = 250


class ModelNetDataset(data.Dataset):
    def __init__(self, root, npoints=2500, split='train', data_augmentation=True):
        self.npoints = npoints
        self.root = root
        self.split = split
        self.data_augmentation = data_augmentation
        self.fns = []
        with open(os.path.join(root, '{}.txt'.format(self.split)), 'r') as f:
            for line in f:
                self.fns.append(line.strip())

        self.cat = {}
        with open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modelnet_id.txt'), 'r') as f:
            for line in f:
                ls = line.strip().split()
                self.cat[ls[0]] = int(ls[1])

        print(self.cat)
        self.classes = list(self.cat.keys())

    def __getitem__(self, index):
        fn = self.fns[index]
        fn = fn.rsplit('_', 1)[0] + '/' + fn + '.txt'
        cls = self.cat[fn.split('/')[0]]
        with open(os.path.join(self.root, fn), 'r') as f:
            pts = self.data_reader(f)
        choice = np.random.choice(len(pts), self.npoints, replace=True)
        point_set = pts[choice, :]

        point_set = point_set - np.expand_dims(np.mean(point_set, axis=0), 0)  # center
        dist = np.max(np.sqrt(np.sum(point_set ** 2, axis=1)), 0)
        point_set = point_set / dist  # scale

        if self.data_augmentation:
            theta = np.random.uniform(0, np.pi * 2)
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            point_set[:, [0, 2]] = point_set[:, [0, 2]].dot(rotation_matrix)  # random rotation
            point_set += np.random.normal(0, 0.02, size=point_set.shape)  # random jitter

        point_set = torch.from_numpy(point_set.astype(np.float32))
        cls = torch.from_numpy(np.array([cls]).astype(np.int64))
        return point_set, cls

    def __len__(self):
        return len(self.fns)

    def data_reader(self, stream):
        lists = stream.readlines()
        points = np.zeros((len(lists), 3), 'float')
        for i in range(len(lists)):
            xyz = lists[i].strip().split(',')
            points[i] = xyz[0], xyz[1], xyz[2]

        return points


def train():
    # ModelNet数据集
    train_dataset = ModelNetDataset(root=dataset, npoints=num_points, split='modelnet40_train')
    test_dataset = ModelNetDataset(root=dataset, npoints=num_points, split='modelnet40_test', data_augmentation=False)
    traindataloader = torch.utils.data.DataLoader(train_dataset, batch_size=batchSize, shuffle=True, num_workers=4)
    testdataloader = torch.utils.data.DataLoader(test_dataset, batch_size=batchSize, shuffle=True, num_workers=4)
    print('Train: ', len(train_dataset), 'Test: ', len(test_dataset))

    # 网络训练参数设置
    num_classes = len(train_dataset.classes)
    classifier = PointNet(num_classes=num_classes)
    optimizer = optim.Adam(classifier.parameters(), lr=0.001, betas=(0.9, 0.999))
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.5)
    classifier.cuda()

    # 训练网络
    train_loss_values = []
    train_accu_values = []
    test_loss_values = []
    test_accu_values = []
    for epoch in range(nepoch):
        scheduler.step()
        total_correct = 0
        total_loss = 0
        for i, data in enumerate(traindataloader, 0):
            points, target = data
            target = target[:, 0]
            points = points.transpose(2, 1)
            points, target = points.cuda(), target.cuda()
            optimizer.zero_grad()
            classifier = classifier.train()
            pred = classifier(points)

            loss = F.nll_loss(pred, target, reduction='sum')
            loss.backward()
            optimizer.step()

            pred_choice = pred.data.max(1)[1]
            correct = pred_choice.eq(target.data).cpu().sum()
            total_correct += correct.item()
            total_loss += loss.item()

            print('[%d: %d/%d]' % (epoch, i, len(train_dataset) / batchSize))

        # 评估当前训练accuracy
        accuracy = total_correct / float(len(train_dataset))
        total_loss = total_loss / float(len(train_dataset))
        train_accu_values.append(accuracy)
        train_loss_values.append(total_loss)
        print('Epoch %d Train: loss: %f, accuracy: %f' % (epoch, total_loss, accuracy))

        # 保存当前网络权重
        torch.save(classifier.state_dict(), 'cls_model_%d.pth' % (epoch))

        # 评估测试accuracy
        total_correct = 0
        total_loss = 0
        for i, data in tqdm(enumerate(testdataloader, 0)):
            points, target = data
            target = target[:, 0]
            points = points.transpose(2, 1)
            points, target = points.cuda(), target.cuda()
            classifier = classifier.eval()
            pred = classifier(points)

            loss = F.nll_loss(pred, target, reduction='sum')

            pred_choice = pred.data.max(1)[1]
            correct = pred_choice.eq(target.data).cpu().sum()
            total_correct += correct.item()
            total_loss += loss.item()

        accuracy = total_correct / float(len(test_dataset))
        total_loss = total_loss / float(len(test_dataset))
        test_accu_values.append(accuracy)
        test_loss_values.append(total_loss)
        print('Epoch %d Test: loss: %f, accuracy: %f' % (epoch, total_loss, accuracy))

    # 绘制loss/accuracy曲线
    plt.plot(train_loss_values, color='blue', label='train')
    plt.plot(test_loss_values, color='green', label='test')
    plt.xlim(0, nepoch)
    plt.xlabel('epoch')
    plt.ylabel('loss')
    plt.title('model loss')
    plt.legend()
    plt.savefig('loss.png')
    plt.close()

    plt.plot(train_accu_values, color='blue', label='train')
    plt.plot(test_accu_values, color='green', label='test')
    plt.xlim(0, nepoch)
    plt.ylim(0, 1)
    plt.xlabel('epoch')
    plt.ylabel('loss')
    plt.title('model accuracy')
    plt.legend()
    plt.savefig('accu.png')


if __name__ == '__main__':
    train()
