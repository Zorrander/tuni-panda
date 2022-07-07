import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F


from .encoder_layer import EncoderLayer
from .feed_forward import FeedForward
from .layer_norm import LayerNorm
from .multi_head_attention import MultiHeadAttention
from .utils import clones


class Encoder(nn.Module):
    """Core encoder is a stack of N layers"""

    def __init__(self, layer: EncoderLayer, N: int):
        super(Encoder, self).__init__()
        self.layers = clones(layer, N)
        self.norm = LayerNorm(layer.size)

    def forward(self, x: torch.FloatTensor, mask: torch.ByteTensor) -> torch.FloatTensor:
        """Pass the input (and mask) through each layer in turn."""
        for layer in self.layers:
            x = layer(x, mask)
        return self.norm(x)



class WeightSum(nn.Module):
    def __init__(self, seq_len, d_model, avg=False):
        super(WeightSum, self).__init__()
        self.avg = avg
        if not avg:
            self.weight = nn.Parameter(torch.FloatTensor(np.random.randn(d_model)), requires_grad=True)
        else:
            self.avg_pool = nn.AvgPool2d(kernel_size=(seq_len, 1))

    def forward(self, x, lengths):
        # x: [batch_size, seq_len, d_model]
        # lengths: [batch_size]
        for b_id, cur_len in enumerate(lengths):
            x[b_id, cur_len:, :] = 0.0 # mask
        #print(x)

        if not self.avg:
            attention_context = torch.matmul(self.weight, x.permute(0, 2, 1)) # [batch_size, seq_len]
            attention_context = attention_context.masked_fill_(attention_context==0.0, -1e10) # mask
            attention_w = F.softmax(attention_context, dim=-1) # [batch_size, seq_len]
            attention_w = attention_w.unsqueeze(dim=1) # [batch_size, 1, seq_len]
            out = torch.bmm(attention_w, x)  #[batch_size, 1, d_model] 
            out = out.squeeze(dim=1)  #[batch, d_model]
        else:
            out = self.avg_pool(x).squeeze(1)
        return out


class ClassifierLayer(nn.Module):
    def __init__(self, d_model, hidden_size, num_classes, dropout=0.0):
        super(ClassifierLayer, self).__init__()
        self.fc1 = nn.Linear(d_model, hidden_size)
        self.fc2 = nn.Linear(hidden_size, num_classes)
        self.act_func = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout(dropout)
        
    def forward(self, x):
        # x: [batch_size, d_model]
        out = self.fc1(x) # [batch_size, hidden_size]
        out = self.act_func(out)
        out = self.dropout(out)
        out = self.fc2(out) # [batch_size, num_classes]
        out = F.softmax(out, dim=-1)
        return out


class RegressionLayer(nn.Module):
    def __init__(self, d_model, hidden_size, num_classes, dropout=0.0):
        super(RegressionLayer, self).__init__()
        self.fc1 = nn.Linear(d_model, hidden_size)
        self.fc2 = nn.Linear(hidden_size, num_classes)
        self.act_func = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout(dropout)
        
    def forward(self, x):
        # x: [batch_size, d_model]
        out = self.fc1(x) # [batch_size, hidden_size]
        out = self.act_func(out)
        out = self.dropout(out)
        out = self.fc2(out) # [batch_size, num_classes]
        out = torch.tanh(out)
        return out

class TransformerEncoder(nn.Module):
    """The encoder of transformer

    Args:
        `n_layers`: number of stacked encoder layers
        `d_model`: model dimension
        `d_ff`: hidden dimension of feed forward layer
        `n_heads`: number of heads of self-attention
        `dropout`: dropout rate, default 0.1
    """

    def __init__(self, d_model: int, d_ff: int, n_heads: int = 1, n_layers: int = 1,
                 dropout: float = 0.1,
                 seq_len: int = 10,
                 num_classes: int = 3):
        super(TransformerEncoder, self).__init__()
        self.multi_headed_attention = MultiHeadAttention(n_heads, d_model, dropout)
        self.feed_forward = FeedForward(d_model, d_ff, dropout)
        self.encoder_layer = EncoderLayer(d_model, self.multi_headed_attention, self.feed_forward, dropout)
        self.encoder = Encoder(self.encoder_layer, n_layers)
        self.weight_sum = WeightSum(seq_len, d_model)
        self.classifer1 = ClassifierLayer(d_model, d_ff, num_classes, 0.01)
        self.classifer2 = ClassifierLayer(d_model, d_ff, num_classes, 0.01)
        self.classifer3 = ClassifierLayer(d_model, d_ff, num_classes, 0.01)
        self.regressor = RegressionLayer(d_model, d_ff, num_classes, 0.01)
    
        self.reset_parameters()

    def reset_parameters(self):
        for p in self.parameters():
            if p.dim() > 1:
                nn.init.xavier_uniform_(p)

    def forward(self, x: torch.FloatTensor, mask: torch.ByteTensor, lengths) -> torch.FloatTensor:
        y = self.encoder(x, mask)
        y = self.weight_sum(y, lengths)
        y = self.regressor(y)
        #print(y.shape)
        return y

    def save(self, PATH):
        torch.save(self.state_dict(), PATH)

    def load(self, PATH, device='cpu'):
        self.load_state_dict(torch.load(PATH, map_location=torch.device(device)))


