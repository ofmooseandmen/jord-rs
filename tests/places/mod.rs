use jord::models::S84Model;
use jord::HorizontalPos;

pub fn copenhagen() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(55.6761, 12.5683)
}

pub fn hassleholm() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(56.1589, 13.7668)
}

pub fn helsingborg() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(56.0465, 12.6945)
}

pub fn hoor() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(55.9349, 13.5396)
}

pub fn kristianstad() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(56.0294, 14.1567)
}

pub fn lund() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(55.7047, 13.1910)
}

pub fn malmo() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(55.6050, 13.0038)
}

pub fn ystad() -> HorizontalPos<S84Model> {
    HorizontalPos::from_s84(55.4295, 13.82)
}
