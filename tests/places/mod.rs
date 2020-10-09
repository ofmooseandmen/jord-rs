use jord::models::S84Model;
use jord::LatLongPos;

pub fn copenhagen() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(55.6761, 12.5683)
}

pub fn hassleholm() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(56.1589, 13.7668)
}

pub fn helsingborg() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(56.0465, 12.6945)
}

pub fn hoor() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(55.9349, 13.5396)
}

pub fn kristianstad() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(56.0294, 14.1567)
}

pub fn lund() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(55.7047, 13.1910)
}

pub fn malmo() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(55.6050, 13.0038)
}

pub fn ystad() -> LatLongPos<S84Model> {
    LatLongPos::from_s84(55.4295, 13.82)
}
