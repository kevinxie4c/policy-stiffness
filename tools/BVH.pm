package BVH;
use parent 'Mocap::BVH';
use Math::Trig qw(:all);
use GLM;

use strict;
use warnings;

my $identity_mat = GLM::Mat4->new(
    1, 0, 0, 0, 
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
);

sub set_position {
    my $this = shift;
    for ($this->joints) {
        $_->{position} = [splice(@_, 0, scalar($_->channels))];
    }
}

sub update_transform {
    my $this = shift;
    my $model_matrix = GLM::Mat4->new($identity_mat);
    $this->update_joint_transform($this->root, $model_matrix);
}

my %axis = (
    X => GLM::Vec3->new(1, 0, 0),
    Y => GLM::Vec3->new(0, 1, 0),
    Z => GLM::Vec3->new(0, 0, 1),
);

sub update_joint_transform {
    my ($this, $joint, $model_matrix) = @_;
    my @channels = $joint->channels;
    my @positions = @{$joint->{position}};
    my $offset = GLM::Vec3->new($joint->offset);
    $model_matrix = GLM::Functions::translate($model_matrix, $offset);
    if (@channels == 6) {
        if ($channels[0] eq 'Xposition' && $channels[1] eq 'Yposition' && $channels[2] eq 'Zposition') {
            my $v = GLM::Vec3->new(@positions[0..2]);
            $model_matrix = GLM::Functions::translate($model_matrix, $v);
            for (my $i = 3; $i < 6; ++$i) {
                if ($channels[$i] =~ /([XYZ])rotation/) {
                    $model_matrix = GLM::Functions::rotate($model_matrix, deg2rad($positions[$i]), $axis{$1});
                } else {
                    die "$channels[$i]: not rotation?";
                }
            }
        } else {
            die 'not position?';
        }
    } else {
        for (my $i = 0; $i < 3; ++$i) {
            if ($channels[$i] =~ /([XYZ])rotation/) {
                $model_matrix = GLM::Functions::rotate($model_matrix, deg2rad($positions[$i]), $axis{$1});
            } else {
                die "$channels[$i]: not rotation?";
            }
        }
    }
    $joint->{transform} = $model_matrix;
    for ($joint->children) {
        $this->update_joint_transform($_, $model_matrix);
    }
}
