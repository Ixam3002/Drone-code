float borner_valeur(float valeur) {
    if (valeur < 0.0f) {
        return 0.0f;
    } 
    if (valeur > 1800.0f) {
        return 1800.0f;
    }
    return valeur;
}