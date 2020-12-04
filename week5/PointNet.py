import torch
import torch.nn as nn
import torch.nn.functional as F


class PointNet(nn.Module):
    def __init__(self, num_classes):
        super(PointNet, self).__init__()

        self.conv1 = nn.Conv1d(3, 64, 1)
        self.bn1 = nn.BatchNorm1d(64)
        self.conv2 = nn.Conv1d(64, 64, 1)
        self.bn2 = nn.BatchNorm1d(64)

        self.conv3 = nn.Conv1d(64, 64, 1)
        self.bn3 = nn.BatchNorm1d(64)
        self.conv4 = nn.Conv1d(64, 128, 1)
        self.bn4 = nn.BatchNorm1d(128)
        self.conv5 = nn.Conv1d(128, 1024, 1)
        self.bn5 = nn.BatchNorm1d(1024)

        self.fc6 = nn.Linear(1024, 512)
        self.bn6 = nn.BatchNorm1d(512)
        self.fc7 = nn.Linear(512, 256)
        self.bn7 = nn.BatchNorm1d(256)
        self.fc8 = nn.Linear(256, num_classes)
        self.bn8 = nn.BatchNorm1d(num_classes)
        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        # mlp 1
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))

        # mlp 2
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))
        x = F.relu(self.bn5(self.conv5(x)))

        # max pool
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)

        # mlp 3
        x = F.relu(self.bn6(self.fc6(x)))
        x = F.relu(self.bn7(self.dropout(self.fc7(x))))
        x = self.fc8(x)

        return F.log_softmax(x, dim=1)
